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
#include <SunUPFLOW.h>
#include <Ticker.h>    //定时中断
#include "FastLED.h"   //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED
#include "OneButton.h" //点击这里会自动打开管理库页面: http://librarymanager/All#OneButton
//https://blog.csdn.net/finedayforu/article/details/108769901
//https://blog.csdn.net/DOF526570/article/details/128943669

#include "Wire.h"      //https://blog.csdn.net/xq151750111/article/details/115142727
// #define DEBUG_SERIAL
// Arduino -Wire库始终使用的是7位地址 最大到0x7f
// Wire库的实现使用了32字节缓冲区
#define WMR_I2C_ADDR 0x78 // 移动机器人从设备地址，可以设置成0 ~ 127中的地址

#define NUM_LEDS 16      // LED灯珠数量
#define LED_PIN 13       // Arduino输出控制信号引脚
#define LED_TYPE WS2812B // LED灯带型号ESP32-S3-DevKitC-1使用SK6822LED芯片 YD:WS2812B
#define COLOR_ORDER GRB  // RGB灯珠中红色、绿色、蓝色LED的排列顺序
#define TOLERANCE_ANGLE_RAD 3e-1

// #define USE_UPFLOW //不使用光流

const float IS_ZERO = 1; // mm
const float ANGLE_IS_ZERO = 0.1;
uint8_t MaxBright = 0;
// LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS];
// 建立光带leds

// Serial2 HWT101
#define IMU_Serial Serial2         // imu串口接口
#define IMU_Serial_BAUDRATE 115200 // HWT101默认115200

// Serial1 LC302GS
#define FLOW_Serial Serial1     // 光流串口接口
#define LC302GS_BAUDRATE 460800 // old 345600

UPFLOW LC302GS(&FLOW_Serial, RXD1, TXD1, LC302GS_BAUDRATE);


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

//车长:280,车宽:240,原点(160,180)
CAR_GOAL_POINT point[] =
    {
        {0, 0, 0},          // p0
        {555, 208, 0}, //1
        {1460, 208, 0}, //2
        {1460, 83.2, 0}, //3
        {1875, 155, 0},    //4    
        {1875, 155, M_PI/2}, //5

        {1950,889,M_PI/2},//6

        {1971,900,M_PI/2}, //7
        {1971,1050,M_PI/2}, //8
        {1971,1200,M_PI/2}, //9

        {1875,1735,M_PI/2}, //10
        {1875,1735,M_PI},  //11

        {1200,1750,M_PI},  //12确保安全

        {1200,1826,M_PI},  //13
        {1050,1826,M_PI},  //14
        { 900,1826,M_PI},   //15

        {1875,1670,M_PI},  //16
        {1875,1670,M_PI/2},  //17
        {1875,1670,M_PI/2}, //18

        {1920,889,M_PI/2},//19

        {1875, 155, M_PI/2},//20
        {1875, 155, 0},//21

        {1460, 75, 0}, //22

        {100, 1734, M_PI},//23
        {0,0,M_PI}//24

};
//校准点
CAR_GOAL_POINT cali_point[] =
{
    {1966,900,M_PI/2},
    {1200,1826,M_PI},
};
// 比例系数和速度最大值
CAR_KPS_MAX botKps{0.01, 0.01, 3, 0.25, 0.15, 10};
// 目标到达标志
bool isXarrived = false;
bool isYarrived = false;
bool isZarrived = false;
//运动顺序
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

// 小车附体坐标系（局部坐标系）下速度
float linear_vel_x = 0;  // m/s
float linear_vel_y = 0;  // m/s
float angular_vel_z = 0; // rad/s

// 光流
float flow_x = 0;                 // mm
float flow_y = 0;                 // mm
float last_flow_x = 0;            // mm
float last_flow_y = 0;            // mm
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
float Kp = 30, Ki = 0.15, Kd = 0;
PID VeloPID[WHEELS_NUM] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

extern PID CAR_ODOM_PID_VX;
extern PID CAR_ODOM_PID_VY;
extern PID CAR_ODOM_PID_VZ;

//*****************创建1个4路电机对象***************************//
BDCMotor motors;

//*****************运行状态标记**************************//

int enum_count = 0;
int print_Count = 0;

int vel_Count = 0;
int imu_Count = 0;
bool brake_on = false;
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
// 定义移动机器人状态变量
char WMR_status = 0x00;
char cmd; // 命令
int8_t f_n, p_n, x_n, y_n, z_n, temp_n;
void onRequest();
void onReceive();
void updateTargetVelocity_0x66(int8_t ponit_n);

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
  botOdometry.calibrationK(1.05, 1.025, 1.0);
  Serial.begin(115200);
  // HWT101陀螺仪采集串口2,默认IMU_Serial_BAUDRATE
  IMU_Serial.begin(IMU_Serial_BAUDRATE, SERIAL_8N1, RXD2, TXD2); // 二合一版
  // Wire初始化, 加入i2c总线
  // 以从机身份加入总线。
  Wire.begin(WMR_I2C_ADDR);
  Wire.onReceive(onReceive); // 收到数据后，执行onReceive
  Wire.onRequest(onRequest); // 收到需求指令，执行onRequest
  start_btn.reset();         // 清除一下按钮状态机的状态
  start_btn.attachClick(start_click);

#ifdef USELED
  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS); // 初始化LED灯
  FastLED.setBrightness(MaxBright);                             // 设置光带亮度
  fill_solid(leds, NUM_LEDS, CRGB::White);                       // 将LED光带设置为同一颜色
  FastLED.show();
#endif

  Serial.println("Sunnybot 麦轮走P2P测试，请按下对应按键开始测试");
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
    //空闲时也要不停调整,不要泄劲
    updateTargetVelocity_0x66(p_n);
    getMotorSpeed(); 
    break;
  case 0x11:
    // 更新走点n速度
    updateTargetVelocity(p_n); // 更新移动机器人目标速度

#ifdef USE_UPFLOW
    getFlowXYVelocity(); // 通过光流数据，使用PID控制器得到XY校准速度，仅使用比例环
#endif

    getMotorSpeed();                            // 通过逆运动学得到电机转速（脉冲数）
    if ((isXarrived && isYarrived && isZarrived)) // 状态转移条件
    {
      Serial.print("prevmillis");
      Serial.print(previousMillis);
      Serial.print("currmillis");
      Serial.println(millis());
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
  if(WMR_status == 0x00)
  {
    return;
  }
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
    botOdometry.botPosition.position_x = cali_point[f_n].world_x;
    botOdometry.botPosition.position_y = cali_point[f_n].world_y;
    break;
  case 'P':
    p_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x11;
    isXarrived = false;
    isYarrived = false;
    isZarrived = false;
    VeloPID[0].reset();
    VeloPID[1].reset();
    VeloPID[2].reset();
    VeloPID[3].reset();
    CAR_ODOM_PID_VX.reset();
    CAR_ODOM_PID_VY.reset();
    CAR_ODOM_PID_VZ.reset();
#ifdef DEBUG_SERIAL
    Serial.println(p_n);
#endif
    break;
  case 'X':
  if((fabs(botOdometry.botPosition.heading_theta - 0)<TOLERANCE_ANGLE_RAD))
  {
    x_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x22;
  }
  else if((fabs(botOdometry.botPosition.heading_theta - M_PI/2)<TOLERANCE_ANGLE_RAD))
  {
    y_n = Wire.read();
    WMR_status = 0x33;
  }
  else if((fabs(botOdometry.botPosition.heading_theta - M_PI)<TOLERANCE_ANGLE_RAD))
  {
    x_n = -Wire.read();
    WMR_status = 0x22;
  }//车头朝向不同时,行为也不同
#ifdef DEBUG_SERIAL
    Serial.println(x_n);
#endif
    break;
  case 'Y':
  if((fabs(botOdometry.botPosition.heading_theta - 0)<TOLERANCE_ANGLE_RAD))
  {
    y_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x33;
  }
  else if((fabs(botOdometry.botPosition.heading_theta - M_PI/2)<TOLERANCE_ANGLE_RAD))
  {
    x_n = -Wire.read();
    WMR_status = 0x22;
  }
  else if((fabs(botOdometry.botPosition.heading_theta - M_PI)<TOLERANCE_ANGLE_RAD))
  {
    y_n = -Wire.read(); // receive byte as an integer
    WMR_status = 0x33;
  }//车头朝向不同时,校准行为也不同
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
  Wire.write(p_n); // 返回机器人状态
}
// 按轴依次更新速度
//输入参数：目标点
void updateTargetVelocity(int8_t ponit_n)
{

  // 获取最大等待时间
  if (max_time_flag)
  {
    max_time_flag = false;     // 该点不再更新
    previousMillis = millis(); // 更新基准时间
    cpx = botOdometry.botPosition.position_x;
    cpy = botOdometry.botPosition.position_y;
    x_max_time = int(abs(cpx - point[ponit_n].world_x) / botKps.MAX_x + 500);
    y_max_time = int(abs(cpy - point[ponit_n].world_y) / botKps.MAX_y + 500);
    z_max_time = int(abs(botOdometry.botPosition.heading_theta - point[ponit_n].world_angular_z) / botKps.MAX_z + 500);
  }
  unsigned long currentMillis = millis(); // store the current time
  mecanumXbot.getBotVel(point[ponit_n], botKps);

#ifndef USE_UPFLOW
    linear_vel_x = mecanumXbot.botVel.vel_x;                        // m/s
    linear_vel_y = mecanumXbot.botVel.vel_y; // m/s
    angular_vel_z = mecanumXbot.botVel.angular_vel_z;
    if((fabs(botOdometry.botPosition.position_y - point[ponit_n].world_y) < IS_ZERO))
    {
      if(abs(ENC[0].read())<1&&abs(ENC[1].read())<1&&abs(ENC[2].read())<1&&abs(ENC[3].read())<1)
      {
        isYarrived = true; 
      }
    }
    if((fabs(botOdometry.botPosition.position_x - point[ponit_n].world_x) < IS_ZERO))
    {
      if(abs(ENC[0].read())<1&&abs(ENC[1].read())<1&&abs(ENC[2].read())<1&&abs(ENC[3].read())<1)
      {
        isXarrived = true; 
      }
    }
    if ((fabs(botOdometry.botPosition.heading_theta - point[ponit_n].world_angular_z) < angle2rad(ANGLE_IS_ZERO)))
    {
      if(abs(ENC[0].read())<1&&abs(ENC[1].read())<1&&abs(ENC[2].read())<1&&abs(ENC[3].read())<1)
      {
        isZarrived = true; 
      }
    }//让车彻底停下后才停止校准,但是调试过程中发现效果不大
    if ((currentMillis - previousMillis >= x_max_time + y_max_time + z_max_time))
    {
      isYarrived = true;
      isXarrived = true;
      isZarrived = true;
      previousMillis = currentMillis;
    }
#endif

#ifdef USE_UPFLOW
  // 按轴依次输出y x z
  switch (motion_direction)
  {
  case Y:
    if((fabs(botOdometry.botPosition.heading_theta - 0)<TOLERANCE_ANGLE_RAD)||(fabs(botOdometry.botPosition.heading_theta - M_PI)<TOLERANCE_ANGLE_RAD)||(fabs(botOdometry.botPosition.heading_theta + M_PI)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_x = 0;                        // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y; // m/s
    }
    else if((fabs(botOdometry.botPosition.heading_theta - M_PI/2)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_x = mecanumXbot.botVel.vel_x;                        // m/s
      linear_vel_y = 0; // m/s
    }
    angular_vel_z = mecanumXbot.botVel.angular_vel_z;
    if ((fabs(botOdometry.botPosition.position_y - point[ponit_n].world_y) < IS_ZERO)||(currentMillis - previousMillis >= y_max_time))
    {
      isYarrived = true;
      motion_direction = X;
      previousMillis = currentMillis;
      botOdometry.botPosition.position_x = cpx; // 认为X轴没变
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
    }
    break;

  case X:
    // linear_vel_x = mecanumXbot.botVel.vel_x; // m/s
    // linear_vel_y = 0;                        // m/s
    if((fabs(botOdometry.botPosition.heading_theta - 0)<TOLERANCE_ANGLE_RAD)||(fabs(botOdometry.botPosition.heading_theta - M_PI)<TOLERANCE_ANGLE_RAD)||(fabs(botOdometry.botPosition.heading_theta + M_PI)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_x = mecanumXbot.botVel.vel_x;                     // m/s
      linear_vel_y = 0; // m/s
    }
    else if((fabs(botOdometry.botPosition.heading_theta - M_PI/2)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_x = 0;                        // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y; // m/s
    }
    angular_vel_z = mecanumXbot.botVel.angular_vel_z;
    if ((fabs(botOdometry.botPosition.position_x - point[ponit_n].world_x) < IS_ZERO)||(currentMillis - previousMillis >= y_max_time))
    {
      // Serial.println(currentMillis - previousMillis);
      isXarrived = true;
      motion_direction = Z;
      previousMillis = currentMillis;
      botOdometry.botPosition.position_y = point[ponit_n].world_y; // 认为Y轴没变
      LC302GS.x_Offset = 0;
      LC302GS.y_Offset = 0;
    }
    break;

  case Z:
    linear_vel_x = 0; // m/s
    linear_vel_y = 0; // m/s
    angular_vel_z = mecanumXbot.botVel.angular_vel_z;
    if ((fabs(botOdometry.botPosition.heading_theta - point[ponit_n].world_angular_z) < angle2rad(IS_ZERO))||(currentMillis - previousMillis >= z_max_time))
    {
      isZarrived = true;
      motion_direction = Y;
      previousMillis = currentMillis;
    }
    break;
  }
#endif
}

void updateTargetVelocity_0x66(int8_t ponit_n)
{
  // 获取最大等待时间
  unsigned long currentMillis = millis(); // store the current time
  mecanumXbot.getBotVel(point[ponit_n], botKps);
  linear_vel_x = mecanumXbot.botVel.vel_x;                        // m/s
  linear_vel_y = mecanumXbot.botVel.vel_y; // m/s
  angular_vel_z = mecanumXbot.botVel.angular_vel_z;
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
  // 光流更新
  flow_x = LC302GS.x_Offset;
  flow_y = LC302GS.y_Offset;
}

// 通过PID控制器得到XY归零矫正速度，仅使用比例环
void getFlowXYVelocity()
{

  switch (motion_direction)
  {
    // 当Y方向运动时，光流校准X方向
  case Y:
    if((fabs(botOdometry.botPosition.heading_theta - 0)<TOLERANCE_ANGLE_RAD)||(fabs(botOdometry.botPosition.heading_theta - M_PI)<TOLERANCE_ANGLE_RAD)||(fabs(botOdometry.botPosition.heading_theta + M_PI)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_x = (-flow_x * 0.00001);
    }
    else if((fabs(botOdometry.botPosition.heading_theta - M_PI/2)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_y = (flow_y * 0.00001);
    }
    break;
    // 当X方向运动时，光流校准Y方向
  case X:
    if((fabs(botOdometry.botPosition.heading_theta - 0)<TOLERANCE_ANGLE_RAD)||(fabs(botOdometry.botPosition.heading_theta - M_PI)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_y = (flow_y * 0.000005);
    }
    else if((fabs(botOdometry.botPosition.heading_theta - M_PI/2)<TOLERANCE_ANGLE_RAD))
    {
      linear_vel_x = (-flow_x * 0.000005);
    }
    break;
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

// 伪串口2中断 更新角度 相位平移
void serialEvent2()
{
  lastYaw = newYawRad; // 保存上一次的值
  while (IMU_Serial.available())
  {
    JY901.CopeSerialData(IMU_Serial.read()); // Call JY901 data cope function
  }
  newYawRad = (float)JY901.stcAngle.Angle[2] / 32768 * M_PI;
  // Serial.println(newYawRad);
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