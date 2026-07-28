
//走矩形线路测试 使用枚举类型
#include <Arduino.h>
#include "OOPConfig.h"

#ifdef USE_MPU6050_DMP
//使用带DMP的支持ESP32的MPU6050库
#include "I2Cdev.h"                         //点击自动打开管理库页面并安装: http://librarymanager/All#MPU6050 ，需安装github网站上的版本
#include "MPU6050_6Axis_MotionApps_V6_12.h" //已和I2Cdev.h同时安装,注意：原版I2Cdev无法使用
MPU6050 mpu;
#endif

#include <Wire.h>
#include <Ticker.h> //定时中断

#define DEBUG
//****************** MPU6050***************************//
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success,
                        // !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 gy; // [x, y, z]            gyro sensor measurements
VectorInt16
    aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld;         // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//小车偏航角
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
    //pid控制器得到 输出PWM
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup()
{

  motors.init();
  motors.flipMotors(
      FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2],
      FLIP_MOTOR[3]); //根据实际转向进行调整false or true 黑色PCB电机
                      // false  绿色PCB电机true 翻转信息包含在OOPConfig
  for (int i = 0; i < WHEELS_NUM; i++)
  {
    ENC[i].init();
    ENC[i].flipEncoder(FLIP_ENCODER[i]);
  }
  delay(100);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(BAUDRATE);

  //****************** MPU6050***************************//
  //陀螺仪初始化
  Wire.begin();
  // Wire.setClock(400000);
  while (!Serial)
    ;
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  //运行例程中的IMU_Zero获得，注意波特率一致问题
  mpu.setXAccelOffset(XAccelOffset);
  mpu.setYAccelOffset(YAccelOffset);
  mpu.setZAccelOffset(ZAccelOffset);
  mpu.setXGyroOffset(XGyroOffset);
  mpu.setYGyroOffset(YGyroOffset);
  mpu.setZGyroOffset(ZGyroOffset);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
    //****************** MPU6050***************************//

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(100); //延时等待初始化完成

  Serial.println("Sunnybot 麦轮走长方形测试，请按下对应按键开始测试");
  while (digitalRead(BUTTON_PIN) == HIGH)
  {
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
  }
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
  //****************** MPU6050***************************//
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  newYawRad = -ypr[0]; //传感器库返回方向和定义方向相反，所以取负号，返回值为弧度
  //第一次运行时把当前角度作为初始值
  if (isFirst)
  {
    initialYawRad = newYawRad;
    isFirst = 0;
  }
  realYawRad = newYawRad - initialYawRad;//偏航角偏差量（弧度）
  realYaw = rad2angle(realYawRad);//偏航角偏差量（角度）
  initialYaw = rad2angle(initialYawRad);
  newYaw = rad2angle(newYawRad);
}

//通过PID控制器得到角速度，仅使用比例环
void getPIDAngularVelocity()
{
  //角速度比例环 vz=k*errZ; errZ=期望-实际
  angular_vel_z = -realYaw * 0.4;
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
  if (print_Count >= 50) //打印控制，控制周期5000ms
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
    print_Count = 0;
  }
}