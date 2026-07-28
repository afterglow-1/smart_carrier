
//走矩形线路测试 使用枚举类型
//bug：millis计时受到影响
//陀螺仪采集串口1
#include <Arduino.h>
#include "OOPConfig.h"
#include <Ticker.h> //定时中断


#define DEBUG

//小车偏航角
//    初始角度，    初始角度弧度， 最新角度，最新角度弧度，偏离角度，偏离角度弧度
float initialYaw, initialYawRad, newYaw, newYawRad, realYaw, realYawRad;
bool isFirst = 1;
float angle2rad(float angle)
{
  return angle * M_PI / 180;
}
float rad2angle(float rad)
{
  return rad * 180 / M_PI;
}
/*
 Kinematics(int motor_max_rpm, float wheel_diameter, float fr_wheels_dist,
float lr_wheels_dist, int pwm_bits);
motor_max_rpm = motor's maximum rpm 电机最大转速
wheel_diameter = robot's wheel diameter expressed in meters 车轮直径
fr_wheels_dist FR_WHEELS_DISTANCE 轴距
lr_wheels_dist LR_WHEELS_DISTANCE = distance between two wheels expressed in
 meters 轮距 pwm_bits = microcontroller's PWM pin resolution. Arduino Uno/Mega
 Teensy is using 8 bits(0-255)
*/
//新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;           // m/s
float linear_vel_y = 0;           // m/s
float angular_vel_z = 0;          // rad/s
unsigned long previousMillis = 0; // will store last time run
const long period = 5000;         // period at which to run in ms
/***************** 定时中断参数 *****************/
Ticker timer1; // 定时中断函数
bool timer_flag = 0;
//******************创建4个编码器实例***************************//
SunEncoder ENC[WHEELS_NUM] = {
    SunEncoder(M1ENA, M1ENB), SunEncoder(M2ENA, M2ENB),
    SunEncoder(M3ENA, M3ENB), SunEncoder(M4ENA, M4ENB)};

long targetPulses[WHEELS_NUM] = {0, 0, 0, 0};   //四个车轮的目标计数
long feedbackPulses[WHEELS_NUM] = {0, 0, 0, 0}; //四个车轮的定时中断编码器四倍频计数
double outPWM[WHEELS_NUM] = {0, 0, 0, 0};

//*****************创建4个速度PID实例***************************//
/*PID(float min_val, float max_val, float kp, float ki, float kd)
 * float min_val = min output PID value
 * float max_val = max output PID value
 * float kp = PID - P constant PID控制的比例、积分、微分系数
 * float ki = PID - I constant
 * float di = PID - D constant
 * Input	(double)输入参数feedbackPulses，待控制的量
 * Output	(double)输出参数outPWM，指经过PID控制系统的输出量
 * Setpoint	(double)目标值targetPulses，希望达到的数值
 */
float Kp = 10, Ki = 0.1, Kd = 0;
PID VeloPID[WHEELS_NUM] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

//*****************创建1个4路电机对象***************************//
BDCMotor motors;

//*****************运行状态标记**************************//
enum CARMOTION
{
  PAUSE,
  LEFTWARD,
  FORWARD,
  RIGHTWARD,
  BACKWARD
};
enum CARMOTION direction = PAUSE;

int print_Count = 0;
//定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR()
{
  //获取电机脉冲数（速度）
  timer_flag = 1; //定时时间达到标志
  print_Count++;
  //  /获取电机目标速度 脉冲计数
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  for (int i = 0; i < WHEELS_NUM; i++)
  {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0); //复位
    // pid控制器得到 输出PWM
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup()
{

  motors.init();
  motors.flipMotors(
      FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2],
      FLIP_MOTOR[3]); //根据实际转向进行调整false or true ，翻转信息包含在OOPConfig
  for (int i = 0; i < WHEELS_NUM; i++)
  {
    ENC[i].init();
    ENC[i].flipEncoder(FLIP_ENCODER[i]);
  }
  delay(100);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(BAUDRATE);
  // 陀螺仪采集串口1
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1); //二合一版
 delay(3000);  // 上电后先延迟一会，确保传感器已上电准备完毕
  // 唤醒传感器，并配置好传感器工作参数，然后开启主动上报---------------
  Cmd_03();  // 2 唤醒传感器
  /**
       * 设置设备参数
     * @param accStill    惯导-静止状态加速度阀值 单位dm/s?
     * @param stillToZero 惯导-静止归零速度(单位cm/s) 0:不归零 255:立即归零
     * @param moveToZero  惯导-动态归零速度(单位cm/s) 0:不归零
     * @param isCompassOn 1=需开启磁场 0=需关闭磁场
     * @param barometerFilter 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
     * @param reportHz 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
     * @param gyroFilter    陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
     * @param accFilter     加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
     * @param compassFilter 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
     * @param Cmd_ReportTag 功能订阅标识
     */
  Cmd_12(5, 255, 0, 0, 3, 20, 2, 4, 9, 0x00E0);  // 7 设置设备参数(内容1)
  Cmd_19();  

  Serial.println("Sunnybot 麦轮走长方形测试，请按下对应按键开始测试");
  while (digitalRead(BUTTON_PIN) == HIGH)
  {
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
  }
  previousMillis=millis();//更新基准时间
  /***************** 定时中断 *****************/
  timer1.attach_ms(TIMER_PERIOD, timerISR); // 打开定时器中断
  interrupts();
}

void loop()
{
  // 10ms运行一次，编码器和电机速度10ms更新一次
  if (timer_flag)
  {
    timer_flag = 0;
    updateTargetVelocity();  //有限状态机方式更新移动机器人目标速度
    updateSensors();         //更新MPU6050传感数据
    getPIDAngularVelocity(); //通过PID控制器得到角速度，仅使用比例环
    getMotorSpeed();         //通过逆运动学得到电机转速（脉冲数）
  }

  debugPrint(); //串口调试输出
}

//有限状态机方式更新移动机器人目标速度
void updateTargetVelocity()
{
  unsigned long currentMillis = millis(); // store the current time
  //使用有限状态机方式走正方形
  // PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD
  switch (direction)
  {
  case PAUSE:          //停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    //使用millis函数进行定时控制，代替delay函数
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = LEFTWARD;
    }
    break;
  case LEFTWARD:        //左进
    linear_vel_x = 0;   // m/s
    linear_vel_y = 0.2; // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = FORWARD;
    }
    break;
  case FORWARD:         //前进
    linear_vel_x = 0.2; // m/s
    linear_vel_y = 0;   // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      direction = RIGHTWARD;
    }
    break;
  case RIGHTWARD:        //右进
    linear_vel_x = 0;    // m/s
    linear_vel_y = -0.2; // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = BACKWARD;
    }
    break;
  case BACKWARD:         //后退
    linear_vel_x = -0.2; // m/s
    linear_vel_y = 0;    // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      direction = PAUSE;
    }
    break;
  default:             //停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = PAUSE;
    }
    break;
  }
}
//更新传感数据
void updateSensors()
{

  newYawRad=angle2rad(newYaw);//转换为弧度
  //第一次运行时把当前角度作为初始值
  if (isFirst)
  {
    initialYawRad = newYawRad;
    isFirst = 0;
  }
  realYawRad = newYawRad - initialYawRad; //偏航角偏差量（弧度）
  realYaw = rad2angle(realYawRad);        //偏航角偏差量（角度）
  initialYaw = rad2angle(initialYawRad);
  //newYaw = rad2angle(newYawRad);
}

//通过PID控制器得到角速度，仅使用比例环
void getPIDAngularVelocity()
{
  //角速度比例环 vz=k*errZ; errZ=期望-实际
  angular_vel_z = -realYaw * 0.2;
}
//通过逆运动学得到电机转速（脉冲数）
void getMotorSpeed()
{
  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  // rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}

//串口调试输出
void debugPrint()
{
  if (print_Count >= 50) //打印控制，控制周期500ms
  {
    //串口输出目标值
    Serial.print(" FL: ");
    Serial.print(pluses.motor1);
    Serial.print(",");
    Serial.print(" FR: ");
    Serial.print(pluses.motor2);
    Serial.print(",");
    Serial.print(" RL: ");
    Serial.print(pluses.motor3);
    Serial.print(",");
    Serial.print(" RR: ");
    Serial.println(pluses.motor4);
    //串口输出反馈值
    Serial.print(" FLF: ");
    Serial.print(feedbackPulses[0]);
    Serial.print(",");
    Serial.print(" FRF: ");
    Serial.print(feedbackPulses[1]);
    Serial.print(",");
    Serial.print(" RLF: ");
    Serial.print(feedbackPulses[2]);
    Serial.print(",");
    Serial.print(" RRF: ");
    Serial.println(feedbackPulses[3]);
    //串口输出 IO输出PWM值
    Serial.print(" FLP: ");
    Serial.print(outPWM[0]);
    Serial.print(",");
    Serial.print(" FRP: ");
    Serial.print(outPWM[1]);
    Serial.print(",");
    Serial.print(" RLP: ");
    Serial.print(outPWM[2]);
    Serial.print(",");
    Serial.print(" RRP: ");
    Serial.println(outPWM[3]);
     //串口输出 yaw
      Serial.println("yaw\t");
    Serial.println(rpy[2]);
    print_Count = 0;
  }
}
//伪串口1中断 处理传感器发过来的数据
void serialEvent1()
{
  while (Serial1.available())
  {
    U8 rxByte = Serial1.read();      // 读取串口的数据
    Cmd_GetPkt(rxByte);           // 移植 每收到1字节数据都填入该函数，当抓取到有效的数据包就会回调进入 Cmd_RxUnpack(U8 *buf, U8 DLen) 函数处理
  }
  newYaw= rpy[2];//更新角度
}
