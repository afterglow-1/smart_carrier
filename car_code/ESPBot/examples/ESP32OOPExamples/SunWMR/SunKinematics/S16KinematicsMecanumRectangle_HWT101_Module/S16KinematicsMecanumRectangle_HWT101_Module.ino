
// 走矩形线路测试 使用枚举类型
// bug：millis计时受到影响？
//添加OLED  IMU  不用IMU
// todo 加速度融合

#include "OOPConfig.h"
#include <JY901.h>
#include <Wire.h>
#include <Ticker.h>                                         //定时中断
// Serial2 HWT101
// Serial1 LC302GS
#define USE_OLED
#ifdef USE_OLED
#include <U8g2lib.h> //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
    /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
#endif
#define IMU_Serial Serial2 //imu串口接口

#define DEBUG

// 小车偏航角
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

// 新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;           // m/s
float linear_vel_y = 0;           // m/s
float angular_vel_z = 0;          // rad/s

unsigned long previousMillis = 0; // will store last time run
const long period = 5000;         // period at which to run in ms
const long stop_time = 1000;      // period at which to run in ms
/***************** 定时中断参数 *****************/
Ticker timer1; // 定时中断函数
bool timer_flag = 0;
//******************创建4个编码器实例***************************//
SunEncoder ENC[WHEELS_NUM] = {
    SunEncoder(M1ENA, M1ENB), SunEncoder(M2ENA, M2ENB),
    SunEncoder(M3ENA, M3ENB), SunEncoder(M4ENA, M4ENB)};

long targetPulses[WHEELS_NUM] = {0, 0, 0, 0};   // 四个车轮的目标计数
long feedbackPulses[WHEELS_NUM] = {0, 0, 0, 0}; // 四个车轮的定时中断编码器四倍频计数
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
  PAUSE = 0,
  LEFTWARD,
  FORWARD,
  RIGHTWARD,
  BACKWARD,
  STOP
};
enum CARMOTION direction = PAUSE;
int enum_count = 0;
int print_Count = 0;

int vel_Count = 0;
int imu_Count = 0;
// 定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR()
{
  // 获取电机脉冲数（速度）
  timer_flag = 1; // 定时时间达到标志
  print_Count++;
  vel_Count++;//控制周期控制
  imu_Count++;
  //  /获取电机目标速度 脉冲计数
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  for (int i = 0; i < WHEELS_NUM; i++)
  {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0); // 复位
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
      FLIP_MOTOR[3]); // 根据实际转向进行调整false or true 黑色PCB电机
                      //  false  绿色PCB电机true 翻转信息包含在OOPConfig
  for (int i = 0; i < WHEELS_NUM; i++)
  {
    ENC[i].init();
    ENC[i].flipEncoder(FLIP_ENCODER[i]);
  }
  delay(100);
#ifdef USE_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();              // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf); // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("TEST press47");
  u8g2.sendBuffer();
#endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(BAUDRATE);
  // HWT101陀螺仪采集串口1,默认115200
  IMU_Serial.begin(115200, SERIAL_8N1, RXD2, TXD2); // 二合一版


  Serial.println("Sunnybot 麦轮走长方形测试，请按下对应按键开始测试");
  while (digitalRead(BUTTON_PIN) == HIGH)
  {
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
  }
  previousMillis = millis(); // 更新基准时间
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
    updateTargetVelocity(); // 有限状态机方式更新移动机器人目标速度
    updateSensors();        // 更新IMU传感数据
    getPIDAngularVelocity(); // 通过PID控制器得到角速度，仅使用比例环
    getMotorSpeed();         // 通过逆运动学得到电机转速（脉冲数）
  }

  debugPrint(); // 串口调试输出
}

// 有限状态机方式更新移动机器人目标速度
void updateTargetVelocity()
{
  unsigned long currentMillis = millis(); // store the current time

  // 使用有限状态机方式走正方形
  //  PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD
  switch (direction)
  {
  case PAUSE:          // 停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    // 使用millis函数进行定时控制，代替delay函数
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      //LC302GS.x_Offset = 0;
      //LC302GS.y_Offset = 0;
      direction = STOP;
    }
    break;
  case LEFTWARD:        // 左进
    linear_vel_x = 0;   // m/s
    linear_vel_y = 0.2; // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      //LC302GS.x_Offset = 0;
      //LC302GS.y_Offset = 0;
      direction = STOP;
    }
    break;
  case FORWARD:         // 前进
    linear_vel_x = 0.2; // m/s
    linear_vel_y = 0;   // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      //LC302GS.x_Offset = 0;
      //LC302GS.y_Offset = 0;
      direction = STOP;
    }
    break;
  case RIGHTWARD:        // 右进
    linear_vel_x = 0;    // m/s
    linear_vel_y = -0.2; // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      //LC302GS.x_Offset = 0;
      //LC302GS.y_Offset = 0;
      direction = STOP;
    }
    break;
  case BACKWARD:         // 后退
    linear_vel_x = -0.2; // m/s
    linear_vel_y = 0;    // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      //LC302GS.x_Offset = 0;
      //LC302GS.y_Offset = 0;
      direction = STOP;
    }
    break;
  case STOP:           // 停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    // 使用millis函数进行定时控制，代替delay函数
    if (currentMillis - previousMillis >= stop_time)
    {
      previousMillis = currentMillis;
     // LC302GS.x_Offset = 0;
      //LC302GS.y_Offset = 0;
      if (++enum_count <= 5)
      {
        direction = (enum CARMOTION)enum_count;
      }
      else
      {
        enum_count = 0;
      }
    }
    break;
  default:             // 停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      //LC302GS.x_Offset = 0;
      //LC302GS.y_Offset = 0;
      direction = PAUSE;
    }
    break;
  }
}
// 更新传感数据
void updateSensors()
{

  // 第一次运行时把当前角度作为初始值
  if (isFirst)
  {
    initialYawRad = newYawRad;
    isFirst = 0;
  }
  realYawRad = newYawRad - initialYawRad; // 偏航角偏差量（弧度）
  realYaw = rad2angle(realYawRad);        // 偏航角偏差量（角度）
  initialYaw = rad2angle(initialYawRad);
  newYaw = rad2angle(newYawRad);
   //flow_x = LC302GS.x_Offset;
      //flow_y = LC302GS.y_Offset;
}


// 通过PID控制器得到角速度，仅使用比例环
void getPIDAngularVelocity()
{
  if (imu_Count >= 1)
  {
    imu_Count = 0;
    // 角速度比例环 vz=k*errZ; errZ=期望-实际
    angular_vel_z = -realYaw * 0.1; // 系数需要根据角度刷新率修改
  }
}
// 通过逆运动学得到电机转速（脉冲数）
void getMotorSpeed()
{
  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  // rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}

// 串口调试输出
void debugPrint()
{
  if (print_Count >= 50) // 打印控制，控制周期1000ms
  {
    // 串口输出目标值
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
    // 串口输出反馈值
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
    // 串口输出 IO输出PWM值
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
    Serial.print(" yaw: ");
    Serial.println(newYaw);
    Serial.print(" flow_x: ");
    Serial.println(flow_x);
    Serial.print(" flow_y: ");
    Serial.println(flow_y);

#ifdef USE_OLED
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB12_tf); // choose a suitable font
    u8g2.setCursor(0, 16);
    u8g2.print("X:");
    u8g2.print(flow_x / 1000);
    u8g2.setCursor(0, 32);
    u8g2.print("Y:");
    u8g2.print(flow_y / 1000);
    u8g2.setCursor(0, 48);
    u8g2.print("yaw:");
    u8g2.print(realYaw);
    u8g2.setCursor(0, 64);
    u8g2.print("M:");
    u8g2.print(enum_count);
    u8g2.sendBuffer();
#endif
    print_Count = 0;
  }
}


// 伪串口2中断 更新角度
void serialEvent2()
{
  while (IMU_Serial.available())
  {
    JY901.CopeSerialData(IMU_Serial.read()); // Call JY901 data cope function
  }
  newYawRad = (float)JY901.stcAngle.Angle[2] / 32768 * M_PI;
}
