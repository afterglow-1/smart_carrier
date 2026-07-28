// ESP32 S3 做为从机
// 添加一键启动
/*
移动机器人状态码：
0x00  待启动
0x11 走目标点n状态，走完后跳到0x66状态
0x22 修改轮式里程计X值,修改后跳到0x11状态
0x33 修改轮式里程计Y值,修改后跳到0x11状态
0x44 修改轮式里程计Z值,修改后跳到0x11状态
0x66  机器人一键启动后准备就绪
*/
// #include <Arduino.h>
#include "OOPConfig.h"
#include <JY901.h>
#include <Ticker.h>    //定时中断
#include "OneButton.h" //点击这里会自动打开管理库页面: http://librarymanager/All#OneButton
//https://blog.csdn.net/finedayforu/article/details/108769901
#include "Wire.h"      //https://blog.csdn.net/xq151750111/article/details/115142727
#define DEBUG_SERIAL
// Arduino -Wire库始终使用的是7位地址 最大到0x7f
// Wire库的实现使用了32字节缓冲区
#define WMR_I2C_ADDR 0x78 // 移动机器人从设备地址，可以设置成0 ~ 127中的地址
const float IS_ZERO = 0.1; // mm

// Serial2 HWT101
#define IMU_Serial Serial2         // imu串口接口
#define IMU_Serial_BAUDRATE 115200 // HWT101默认115200



// 小车偏航角
float initialYaw, initialYawRad, newYaw, newYawRad, realYaw, realYawRad, lastYaw;
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
// 新建小车里轮式程计实例
WheelOdometry botOdometry(&kinematics);
// 新建小车位置环P2P实例
Car mecanumXbot(&botOdometry);
// 目标点结构体数组 X:mm Y:mm Z:弧度
CAR_GOAL_POINT point[] =
    {
        {0, 0, 0},          // p0
        {895, 250, 0}, // p1 扫码点
        {1601.3, 246.4, 0},    // p2 原料点
        {0, 1200, 0}        // p3
};
// 比例系数和速度最大值
CAR_KPS_MAX botKps{0.01, 0.01, 2, 0.3, 0.3, 3};
// 目标到达标志
bool isXarrived = false;
bool isYarrived = false;
bool isZarrived = false;
enum MotionDirection
{
  X,
  Y,
  Z
};
enum MotionDirection motion_direction = Y;
float cpx, cpy; // P2P初始时刻x y位置
bool max_time_flag = true;
int x_max_time, y_max_time, z_max_time;

// 小车附体坐标系（局部坐标系）下目标速度
float linear_vel_x = 0;  // m/s
float linear_vel_y = 0;  // m/s
float angular_vel_z = 0; // rad/s

unsigned long previousMillis = 0; // will store last time run
// 运行周期
const long period = 6000;    // period at which to run in ms
const long stop_time = 1000; // period at which to stop in ms
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
float Kp = 10, Ki = 0.1, Kd = 0;
PID VeloPID[WHEELS_NUM] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

//*****************创建1个4路电机对象***************************//
BDCMotor motors;

//*****************运行状态标记**************************//

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
  vel_Count++; // 速度周期控制
  imu_Count++; // 角度校准控制周期
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
  // 里程计更新小车位姿,位姿态值单位 mm  弧度制
  botOdometry.getPositon_mm(feedbackPulses[0], feedbackPulses[1],
                            feedbackPulses[2], feedbackPulses[3], realYawRad);
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]); 
}

// 按键
const int START_BTN_PIN = 47;
bool start_flag = 0;
// 定义移动机器人状态变量，初始值为0x00
char WMR_status = 0x00;
char cmd; // 命令
int8_t f_n, p_n, x_n, y_n, z_n, temp_n;
void onRequest();
void onReceive();

OneButton start_btn(START_BTN_PIN, true, true); // true:按下为低电平,true上拉模式
void start_click()
{
  start_flag = 1;
  WMR_status = 0x66;
  // digitalWrite(EN_PIN, LOW);
  // save_flag=1;
}
void setup()
{
  motors.init();
  motors.flipMotors(
      FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2],
      FLIP_MOTOR[3]); // 根据实际转向进行调整false or true 黑色PCB电机
                      //  false  绿色PCB电机true 翻转信息包含在OOPConfig
  // 编码器初始化和极性设置
  for (int i = 0; i < WHEELS_NUM; i++)
  {
    ENC[i].init();
    ENC[i].flipEncoder(FLIP_ENCODER[i]);
  }
  delay(100);
  // 更换地面条件，要重新校准
  //                         X    Y    Z
  botOdometry.calibrationK(1.0, 1.0, 1.0);
  botOdometry.setPositon_mm(150.0, 150.0, 0.0);

  Serial.begin(115200);
  // HWT101陀螺仪采集串口2,默认IMU_Serial_BAUDRATE
  IMU_Serial.begin(IMU_Serial_BAUDRATE, SERIAL_8N1, RXD2, TXD2); // 二合一版
  // Wire初始化, 加入i2c0总线
  // 以从机身份加入总线。
  Wire.begin(WMR_I2C_ADDR);
  Wire.onReceive(onReceive); // 收到数据后，执行onReceive
  Wire.onRequest(onRequest); // 收到需求指令，执行onRequest
  start_btn.reset();         // 清除一下按钮状态机的状态
  start_btn.attachClick(start_click);



  Serial.println("手脚通讯控制测试，请按下对应按键开始测试");
  previousMillis = millis(); // 更新基准时间
  /***************** 定时中断 *****************/
  timer1.attach_ms(TIMER_PERIOD, timerISR); // 打开定时器中断
  interrupts();
}

void loop()
{
  // 电机速度10ms更新一次
  if (timer_flag)
  {
    timer_flag = 0;
    updateSensors(); // 更新IMU、光流传感数据
  }
  switch (WMR_status)
  {
    // 待一键启动状态
  case 0x00:
    // keep watching the push button:

    start_btn.tick();

    break;
    // 待机空闲状态
  case 0x66:
    // 通过iic接收命令，切换状态
    break;
  case 0x11:
    // 更新走点n速度
    updateTargetVelocity(p_n); // 更新移动机器人目标速度
    getMotorSpeed();                            // 通过逆运动学得到电机转速（脉冲数）
    if (isXarrived && isYarrived && isZarrived) // 状态转移条件
    {

      WMR_status = 0x66;
      isXarrived = false;
      isYarrived = false;
      isZarrived = false;
      max_time_flag = true; // 下一点开启最大运行时间更新
    }
    break;
  case 0x22:
    // 修改里程计X
    botOdometry.botPosition.position_x -= x_n;
    WMR_status = 0x11;
    break;
  case 0x33:
    // 修改里程计Y
    botOdometry.botPosition.position_y -= y_n;
    WMR_status = 0x11;
    break;
  case 0x44:
    // 修改里程计Z
    botOdometry.botPosition.heading_theta -= z_n;
    WMR_status = 0x11;
    break;
  default:
    WMR_status = 0x66;
    break;
  }
  // keep watching the push button:
  start_btn.tick();
}
// 从主设备收到数据后，执行receiveEvent
void onReceive(int howMany)
{
  // 循环读取数据(除了最后一个字符)
  while (1 < Wire.available()) //
  {
    // 接收字节数据并赋值给变量cmd(char)
    cmd = Wire.read();
// 打印该字节
#ifdef DEBUG_SERIAL
    Serial.print(cmd);
#endif
  }
  // 根据命令，存储对应数据
  switch (cmd)
  {
  case 'F':
    f_n = Wire.read(); // 以uint8整数的形式接受字节数据并赋值给f_n(uint8)
#ifdef DEBUG_SERIAL
    Serial.println(f_n);
#endif
    break;
  case 'P':
    p_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x11;
#ifdef DEBUG_SERIAL
    Serial.println(p_n);
#endif
    break;
  case 'X':
    x_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x22;
#ifdef DEBUG_SERIAL
    Serial.println(x_n);
#endif
    break;
  case 'Y':
    y_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x33;
#ifdef DEBUG_SERIAL
    Serial.println(y_n);
#endif
    break;
  case 'Z':
    z_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x44;
#ifdef DEBUG_SERIAL
    Serial.println(z_n);
#endif
    break;
  default:
    temp_n = Wire.read(); // receive byte as an integer
#ifdef DEBUG_SERIAL
    Serial.println(temp_n);
#endif
    break;
  }
}

// 当收到需求指令的时候，执行requestEvent函数内容
void onRequest()
{
  Wire.write(WMR_status); // 返回机器人状态
}
// 按轴依次更新速度
//输入参数：目标点
void updateTargetVelocity(int8_t ponit_n)
{
  unsigned long currentMillis = millis(); // store the current time
  mecanumXbot.getBotVel(point[ponit_n], botKps);

      linear_vel_x = mecanumXbot.botVel.vel_x;                        // m/s
    linear_vel_y = mecanumXbot.botVel.vel_y; // m/s
    angular_vel_z = mecanumXbot.botVel.angular_vel_z;
        if ((currentMillis - previousMillis >= 15000) )
    {
      isYarrived = true;
      isXarrived = true;
      isZarrived = true;
      previousMillis = currentMillis;
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



// 通过逆运动学得到电机转速（脉冲数）
void getMotorSpeed()
{
  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  // rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}



// 伪串口2中断 更新角度 相位平移
void serialEvent2()
{
  lastYaw = newYawRad; // 保存上一次的值
  while (IMU_Serial.available())
  {
    JY901.CopeSerialData(IMU_Serial.read()); // Call JY901 data cope function
  }
  newYawRad = (float)JY901.stcAngle.Angle[2] / 32768 * M_PI;
  if (fabs(newYawRad - lastYaw) > M_PI) // 如果差值大于M_PI，平移相位 每当连续相位角之间的跳跃大于π 弧度时，通过增加 ±2π 的整数倍来平移相位角，确保跳动小于 π。
  {
    if (lastYaw > 0)
    {
      newYawRad = newYawRad + 2 * M_PI;
    }
    else if (lastYaw < 0)
    {
      newYawRad = newYawRad - 2 * M_PI;
    }
  }
}