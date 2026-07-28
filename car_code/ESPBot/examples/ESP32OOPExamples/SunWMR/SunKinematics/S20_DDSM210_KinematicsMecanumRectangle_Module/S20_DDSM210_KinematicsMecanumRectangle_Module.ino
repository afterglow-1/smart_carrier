
// 走矩形线路测试 使用枚举类型

#include "OOPConfig.h"
#include <JY901.h>
#include <Wire.h>
#include <Ticker.h>                                         //定时中断

#define DDSM_RX 14//18
#define DDSM_TX 27//19


// 新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;

//局部坐标系下速度
float linear_vel_x = 0;           // m/s
float linear_vel_y = 0;           // m/s
float angular_vel_z = 0;          // rad/s

unsigned long previousMillis = 0; // will store last time run
const long period = 5000;         // period at which to run in ms
const long stop_time = 1000;      // period at which to run in ms
/***************** 定时中断参数 *****************/
Ticker timer1; // 定时中断函数
bool timer_flag = 0;

//*****************创建1个DDSM电机对象***************************//
DDSM_CTRL dc4;

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
}

void setup()
{

  Serial.begin(BAUDRATE);
  	// ddsm init.
	Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, DDSM_RX, DDSM_TX);
	dc4.pSerial = &Serial1;
	// config the type of ddsm. 
	dc4.set_ddsm_type(210);
	// clear ddsm serial buffer.
	dc4.clear_ddsm_buffer();
  // HWT101陀螺仪采集串口2,默认115200
  Serial.println("Sunnybot 麦轮走长方形测试，请按下对应按键开始测试");
   previousMillis = millis(); // 更新基准时间
  /***************** 定时中断 *****************/
  timer1.attach_ms(20, timerISR); // 打开定时器中断
  interrupts();
}

void loop()
{
  // 10ms运行一次，编码器和电机速度10ms更新一次
  if (timer_flag)
  {
    timer_flag = 0;
    updateTargetVelocity(); // 有限状态机方式更新移动机器人目标速度
    getMotorSpeed();         // 通过逆运动学得到电机转速（脉冲数）
  }

 // debugPrint(); // 串口调试输出
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
      direction = STOP;
    }
    break;
  case LEFTWARD:        // 左进
    linear_vel_x = 0;   // m/s
    linear_vel_y = 0.5; // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;

      direction = STOP;
    }
    break;
  case FORWARD:         // 前进
    linear_vel_x = 0.5; // m/s
    linear_vel_y = 0;   // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;

      direction = STOP;
    }
    break;
  case RIGHTWARD:        // 右进
    linear_vel_x = 0;    // m/s
    linear_vel_y = -0.5; // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;

      direction = STOP;
    }
    break;
  case BACKWARD:         // 后退
    linear_vel_x = -0.5; // m/s
    linear_vel_y = 0;    // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
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

      direction = PAUSE;
    }
    break;
  }
}




// 通过逆运动学得到电机转速（脉冲数）,并设置
void getMotorSpeed()
{
  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  dc4.ddsm210_ctrl_4(rpm.motor1*10,-rpm.motor2*10,rpm.motor3*10,-rpm.motor4*10);
  //pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}





