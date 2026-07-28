
// 走矩形线路测试 使用枚举类型
// bug：millis计时受到影响
// todo 加速度融合
#include <Arduino.h>
#include "OOPConfig.h"
#include <JY901.h>
#include <SunUPFLOW.h>
#include <Wire.h>
#include <Ticker.h>                                         //定时中断
unsigned char Horizontal[3] = {0xFF, 0xAA, 0x65};           // 模块水平放置
unsigned char resetZAngle[3] = {0xFF, 0xAA, 0x52};          // Z轴角度复位指令
unsigned char unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};   //  解锁寄存器 https://wit-motion.yuque.com/wumwnr/ltst03/vl3tpy?#SzruE
unsigned char setBaud[5] = {0xFF, 0xAA, 0x04, 0x06, 0x00};  //  设置波特率为115200
unsigned char saveData[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00}; //  保存数据
// JY61P https://wit-motion.yuque.com/wumwnr/docs/np25sf?singleDoc#%20%E3%80%8AJY61P%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99%E3%80%8B
//  https://wit-motion.yuque.com/docs/share/35421cdb-d120-4ba1-8647-7cb2dba9beed?#%20%E3%80%8AWT61%E5%8D%8F%E8%AE%AE%E3%80%8B
// https://wit-motion.yuque.com/wumwnr/ltst03/rqyk6g?singleDoc#

// Serial2 JY61P
// Serial1 LC302GS
#define IMU_Serial Serial2
#define FLOW_Serial Serial1

#define DEBUG
UPFLOW LC302GS(&FLOW_Serial, RXD1, TXD1, 345600);
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
// 新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;           // m/s
float linear_vel_y = 0;           // m/s
float angular_vel_z = 0;          // rad/s
float flow_x = 0;                 // mm
float flow_y = 0;                 // mm
float last_flow_x = 0;            // mm
float last_flow_y = 0;            // mm
unsigned long previousMillis = 0; // will store last time run
const long period = 5000;         // period at which to run in ms
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
  PAUSE,
  LEFTWARD,
  FORWARD,
  RIGHTWARD,
  BACKWARD
};
enum CARMOTION direction = PAUSE;

int print_Count = 0;

int vel_Count = 0;
int imu_Count = 0;
// 定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR()
{
  // 获取电机脉冲数（速度）
  timer_flag = 1; // 定时时间达到标志
  print_Count++;
  vel_Count++;
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
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(BAUDRATE);
  // jy61陀螺仪采集串口1,默认9600
  IMU_Serial.begin(9600, SERIAL_8N1, RXD2, TXD2); // 二合一版
  /*
  //尝试修改为115200，失败？
  IMU_Serial.write(unlock, 5);                    //解锁
  IMU_Serial.flush();//等待发送完成
  IMU_Serial.write(setBaud, 5);                    //设置波特率为115200
  IMU_Serial.flush();//等待发送完成
  IMU_Serial.write(saveData, 5);                    //保存
  IMU_Serial.flush();//等待发送完成
  //delay(10);
  //按115200设置
 IMU_Serial.begin(115200, SERIAL_8N1, RXD1, TXD1);  //二合一版
 //IMU_Serial.updateBaudRate(115200);//重新设置波特率

 */
  IMU_Serial.write(Horizontal, 3);  // Z轴角度复位指令
  IMU_Serial.flush();               // 等待发送完成
  IMU_Serial.write(resetZAngle, 3); // Z轴角度复位指令
  IMU_Serial.flush();               // 等待发送完成

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
    updateSensors();        // 更新MPU6050传感数据

    getPIDXYVelocity();      // 通过PID控制器得到XY校准速度，仅使用比例环
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
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
      direction = LEFTWARD;
    }
    break;
  case LEFTWARD:        // 左进
    linear_vel_x = 0;   // m/s
    linear_vel_y = 0.2; // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
      direction = FORWARD;
    }
    break;
  case FORWARD:         // 前进
    linear_vel_x = 0.2; // m/s
    linear_vel_y = 0;   // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
      direction = RIGHTWARD;
    }
    break;
  case RIGHTWARD:        // 右进
    linear_vel_x = 0;    // m/s
    linear_vel_y = -0.2; // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
      direction = BACKWARD;
    }
    break;
  case BACKWARD:         // 后退
    linear_vel_x = -0.2; // m/s
    linear_vel_y = 0;    // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
      direction = PAUSE;
    }
    break;
  default:             // 停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
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
}



// 通过PID控制器得到XY归零矫正速度，仅使用比例环
void getPIDXYVelocity()
{
  if (vel_Count >= 2)
  {
    vel_Count = 0;
    // 如果X值不变，判断条件为速度为0
    if (0 == linear_vel_x)
    {
      linear_vel_x = -flow_x * 0.00001;
    }

    // 如果y值不变，判断条件为速度为0
    if (0 == linear_vel_y)
    {
      linear_vel_y = flow_y * 0.00001;
    }
  }
}

// 通过PID控制器得到角速度，仅使用比例环
void getPIDAngularVelocity()
{
  if (imu_Count >= 10)
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
  if (print_Count >= 50) // 打印控制，控制周期5000ms
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
    print_Count = 0;
  }
}
// 伪串口1中断 更新光流数值
void serialEvent1()
{
  while (FLOW_Serial.available())
  {
    if (UPFLOW_STATUS_SUCCESS == LC302GS.readData(FLOW_Serial.read()))
    {
      flow_x = LC302GS.x_Offset;
      flow_y = LC302GS.y_Offset;

    } // Call
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
