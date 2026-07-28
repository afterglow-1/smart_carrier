
// P2P点位运动/点镇定（Point-to-Point Motion/Pose Stabilization）
// 走四个点测试 使用枚举类型 平移相位角度 可连续多圈旋转
// 添加OLED  IMU  FLOW
// todo 加速度融合
#include <Arduino.h>
#include "OOPConfig.h"
#include <JY901.h>
#include <SunUPFLOW.h>
#include <Wire.h>
#include <Ticker.h>  //定时中断

#include "FastLED.h"  //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED
#define DEBUG         // 是否开启串口调试

#define NUM_LEDS 16       // LED灯珠数量
#define LED_PIN 13        // Arduino输出控制信号引脚
#define LED_TYPE WS2812B  // LED灯带型号ESP32-S3-DevKitC-1使用SK6822LED芯片 YD:WS2812B
#define COLOR_ORDER GRB   // RGB灯珠中红色、绿色、蓝色LED的排列顺序
bool useUpflow = true;
bool useIMU = false;
float complementaryCoef = 1;  // 互补滤波系数；
const float IS_ZERO = 0.0001;
uint8_t MaxBright = 255;
// LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS];
// 建立光带leds

unsigned char Horizontal[3] = { 0xFF, 0xAA, 0x65 };            // 模块水平放置
unsigned char resetZAngle[3] = { 0xFF, 0xAA, 0x52 };           // Z轴角度复位指令
unsigned char unlock[5] = { 0xFF, 0xAA, 0x69, 0x88, 0xB5 };    //  解锁寄存器 https://wit-motion.yuque.com/wumwnr/ltst03/vl3tpy?#SzruE
unsigned char setBaud[5] = { 0xFF, 0xAA, 0x04, 0x06, 0x00 };   //  设置波特率为115200
unsigned char saveData[5] = { 0xFF, 0xAA, 0x00, 0x00, 0x00 };  //  保存数据
// JY61P https://wit-motion.yuque.com/wumwnr/docs/np25sf?singleDoc#%20%E3%80%8AJY61P%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99%E3%80%8B
//  https://wit-motion.yuque.com/docs/share/35421cdb-d120-4ba1-8647-7cb2dba9beed?#%20%E3%80%8AWT61%E5%8D%8F%E8%AE%AE%E3%80%8B
// https://wit-motion.yuque.com/wumwnr/ltst03/rqyk6g?singleDoc#


#define USE_OLED
#ifdef USE_OLED
#include <U8g2lib.h>  //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
  /* data=*/SDA);  // ESP32 Thing, HW I2C with pin remapping
#endif
// Serial2 JY61P
#define IMU_Serial Serial2  // imu串口接口
// Serial1 LC302GS
#define FLOW_Serial Serial1  // 光流串口接口
#define LC302GS_BAUD 460800  // old 345600
#define DEBUG
UPFLOW LC302GS(&FLOW_Serial, RXD1, TXD1, 460800);
// 小车偏航角
float initialYaw, initialYawRad, newYaw, newYawRad, realYaw, realYawRad, lastYaw;
bool isFirst = 1;
float angle2rad(float angle) {
  return angle * M_PI / 180;
}
float rad2angle(float rad) {
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

*/
// 新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;
// 新建小车里轮式程计实例
WheelOdometry botOdometry(&kinematics);
// 新建小车位置环P2P实例
Car mecanumXbot(&botOdometry);
// 目标点结构体 X:mm Y:mm Z:弧度
CAR_GOAL_POINT p0{ 0, 0, 0 };
CAR_GOAL_POINT p1{ 2100, 0, 0 };
CAR_GOAL_POINT p2{ 2100, 0, M_PI/2 };
CAR_GOAL_POINT p3{ 0, 2100, 0 };
CAR_GOAL_POINT p4{ 0, 0, 0 };

// 比例系数和速度最大值
CAR_KPS_MAX botKps{ 0.01, 0.01, 1, 0.4, 0.4, 3 };
//目标到达标志
bool isXarrived = false;
bool isYarrived = false;
bool isZarrived = false;
// 运行状态机状态标记
enum CARMOTION {
  P0,
  P1,
  P2,
  P3,
  P4,
  STOP
};
enum CARMOTION position = P0;

// 小车附体坐标系（局部坐标系）下速度
float linear_vel_x = 0;   // m/s
float linear_vel_y = 0;   // m/s
float angular_vel_z = 0;  // rad/s
//光流
float flow_x = 0;                  // mm
float flow_y = 0;                  // mm
float last_flow_x = 0;             // mm
float last_flow_y = 0;             // mm
unsigned long previousMillis = 0;  // will store last time run
// 运行周期
const long period = 10000;     // period at which to run in ms
const long stop_time = 1000;  // period at which to stop in ms
/***************** 定时中断参数 *****************/
Ticker timer1;  // 定时中断函数
bool timer_flag = 0;
//******************创建4个编码器实例***************************//
SunEncoder ENC[WHEELS_NUM] = {
  SunEncoder(M1ENA, M1ENB), SunEncoder(M2ENA, M2ENB),
  SunEncoder(M3ENA, M3ENB), SunEncoder(M4ENA, M4ENB)
};

long targetPulses[WHEELS_NUM] = { 0, 0, 0, 0 };    // 四个车轮的目标计数
long feedbackPulses[WHEELS_NUM] = { 0, 0, 0, 0 };  // 四个车轮的定时中断编码器四倍频计数
double outPWM[WHEELS_NUM] = { 0, 0, 0, 0 };

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
  PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)
};

//*****************创建1个4路电机对象***************************//
BDCMotor motors;

//*****************运行状态标记**************************//

int enum_count = 0;
int print_Count = 0;

int vel_Count = 0;
int imu_Count = 0;
// 定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR() {
  // 获取电机脉冲数（速度）
  timer_flag = 1;  // 定时时间达到标志
  print_Count++;
  vel_Count++;  // 速度周期控制
  imu_Count++;  // 角度校准控制周期
  //  /获取电机目标速度 脉冲计数
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  for (int i = 0; i < WHEELS_NUM; i++) {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0);  // 复位
    // pid控制器得到 输出PWM
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  // 里程计更新小车位姿,位姿态值单位 mm  弧度制
  botOdometry.getPositon_mm(feedbackPulses[0], feedbackPulses[1],
                            feedbackPulses[2], feedbackPulses[3], realYawRad);
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup() {

  motors.init();
  motors.flipMotors(
    FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2],
    FLIP_MOTOR[3]);  // 根据实际转向进行调整false or true 黑色PCB电机
                     //  false  绿色PCB电机true 翻转信息包含在OOPConfig
  //编码器初始化和极性设置
  for (int i = 0; i < WHEELS_NUM; i++) {
    ENC[i].init();
    ENC[i].flipEncoder(FLIP_ENCODER[i]);
  }
  delay(100);
  //更换地面条件，要重新校准
  //                        X    Y    Z
  botOdometry.calibrationK(1.0048, 0.9762, 1.0);
#ifdef USE_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf);  // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("P2P TEST");
  u8g2.sendBuffer();
#endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(BAUDRATE);
  // jy61陀螺仪采集串口1,默认9600
  IMU_Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);  // 二合一版
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
  IMU_Serial.write(Horizontal, 3);   // Z轴角度复位指令
  IMU_Serial.flush();                // 等待发送完成
  IMU_Serial.write(resetZAngle, 3);  // Z轴角度复位指令
  IMU_Serial.flush();                // 等待发送完成

  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);  // 初始化LED灯
  FastLED.setBrightness(MaxBright);                              // 设置光带亮度
  fill_solid(leds, NUM_LEDS, CRGB::Blue);                        // 将LED光带设置为同一颜色
  FastLED.show();

  Serial.println("Sunnybot 麦轮走P2P测试，请按下对应按键开始测试");
#ifdef USE_OLED
  u8g2.clearBuffer();                          // clear the internal memory
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  // choose a suitable font
  u8g2.setCursor(0, 32);
  u8g2.print("请按对应按键ON");  // 汉字用print
  u8g2.sendBuffer();
#endif
  while (digitalRead(BUTTON_PIN) == HIGH) {
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
  }
  previousMillis = millis();  // 更新基准时间
  /***************** 定时中断 *****************/
  timer1.attach_ms(TIMER_PERIOD, timerISR);  // 打开定时器中断
  interrupts();
}

void loop() {
  // 10ms运行一次，编码器和电机速度10ms更新一次
  if (timer_flag) {
    timer_flag = 0;
    updateTargetVelocity();  // 有限状态机方式更新移动机器人目标速度
    updateSensors();         // 更新IMU传感数据

    //getFlowXYVelocity();  // 通过光流数据，使用PID控制器得到XY校准速度，仅使用比例环
    
    getMotorSpeed();      // 通过逆运动学得到电机转速（脉冲数）
  }

 // debugPrint();  // 串口调试输出
}

// 有限状态机方式更新移动机器人目标速度
void updateTargetVelocity() {
  unsigned long currentMillis = millis();  // store the current time

  // P2P走点位
  switch (position) {
    case P0:  // 第1点
      mecanumXbot.getBotVel(p0, botKps);
      //useUpflow = false;
      linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      // angular_vel_z = 0;  // rad/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      // 使用millis函数进行定时控制，代替delay函数
      if (currentMillis - previousMillis >= 10000) {
        previousMillis = currentMillis;
        botOdometry.botPosition.position_x = p0.world_x;
        botOdometry.botPosition.position_y = p0.world_y;
        //LC302GS.x_Offset = 0;
        //LC302GS.y_Offset = 0;
        position = P1;
      }
      break;
    case P1:  // 第2点
      mecanumXbot.getBotVel(p1, botKps);
      linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      if ((currentMillis - previousMillis >= 10000) ) {
        //motors.motorsBrake();
        //关闭光流
       
        previousMillis = currentMillis;
        botOdometry.botPosition.position_x = p1.world_x;
        botOdometry.botPosition.position_y = p1.world_y;
        position = STOP;
    
      }
      break;

    case STOP:  // STOP点
      //motors.motorsBrake();
      for (int i = 0; i < WHEELS_NUM; i++) {
        VeloPID[i].reset();
      }
     
      linear_vel_x = 0;  // m/s
      linear_vel_y = 0;  // m/s
      angular_vel_z = 0;
      // 使用millis函数进行定时控制，代替delay函数
      if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;

        position = P2;
      }
      break;
    case P2:  // 第3点
      mecanumXbot.getBotVel(p2, botKps);
      linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      if (currentMillis - previousMillis >= (period)) {
        previousMillis = currentMillis;
        botOdometry.botPosition.position_x = p2.world_x;
        botOdometry.botPosition.position_y = p2.world_y;
        position = P3;
      }
      break;
    case P3:  // 前进
      mecanumXbot.getBotVel(p3, botKps);

      // 上一点坐标等于目标点目标，并开启光流时，信任光流，速度为零
      if ((abs(p2.world_x - p3.world_x) < IS_ZERO) && useUpflow) {
        linear_vel_x = 0;
      } else {
        linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      }
      // 上一点坐标等于目标点目标，并开启光流时，信任光流，速度为零
      if ((abs(p2.world_y - p3.world_y) < IS_ZERO) && useUpflow) {
        linear_vel_y = 0;
      } else {
        linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      }

      // angular_vel_z = 0;  // rad/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      if (currentMillis - previousMillis >= (period)) {
        previousMillis = currentMillis;
        botOdometry.botPosition.position_x = p3.world_x;
        botOdometry.botPosition.position_y = p3.world_y;
        LC302GS.x_Offset = 0;
        LC302GS.y_Offset = 0;
        position = P4;
      }
      break;

    case P4:  // 前进
      mecanumXbot.getBotVel(p4, botKps);

      // 上一点坐标等于目标点目标，并开启光流时，信任光流，速度为零
      if ((abs(p3.world_x - p4.world_x) < IS_ZERO) && useUpflow) {
        linear_vel_x = 0;
      } else {
        linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      }
      // 上一点某坐标等于目标点某目标，并开启光流时，信任光流，速度为零
      if ((abs(p3.world_y - p4.world_y) < IS_ZERO) && useUpflow) {
        linear_vel_y = 0;
      } else {
        linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      }

      // angular_vel_z = 0;  // rad/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      if (currentMillis - previousMillis >= (period)) {
        previousMillis = currentMillis;
        botOdometry.botPosition.position_x = p4.world_x;
        botOdometry.botPosition.position_y = p4.world_y;
        LC302GS.x_Offset = 0;
        LC302GS.y_Offset = 0;
        position = P1;
      }
      break;

    default:             // 停止
      linear_vel_x = 0;  // m/s
      linear_vel_y = 0;  // m/s
      angular_vel_z = 0;
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        position = P0;
      }
      break;
  }
}
// 更新传感数据
void updateSensors() {

  // 第一次运行时把当前角度作为初始值
  if (isFirst) {
    initialYawRad = newYawRad;
    isFirst = 0;
  }
  realYawRad = newYawRad - initialYawRad;  // 偏航角偏差量（弧度）
  realYaw = rad2angle(realYawRad);         // 偏航角偏差量（角度）
  initialYaw = rad2angle(initialYawRad);
  newYaw = rad2angle(newYawRad);
  // 光流更新
  flow_x = LC302GS.x_Offset;
  flow_y = LC302GS.y_Offset;
}

// 通过PID控制器得到XY归零矫正速度，仅使用比例环
void getFlowXYVelocity() {  //校准周期和开关
  if (vel_Count >= 2 && useUpflow) {
    vel_Count = 0;
    // 如果X值不变，判断条件为速度为0
    if (0 == linear_vel_x) {
      //linear_vel_x = (-flow_x * 0.00001) * complementaryCoef + (1 - complementaryCoef) * mecanumXbot.botVel.vel_x;
      linear_vel_x = (-flow_x * 0.00001);
    }

    // 如果y值不变，判断条件为速度为0
    if (0 == linear_vel_y) {
      //linear_vel_y = (flow_y * 0.00001) * complementaryCoef + (1 - complementaryCoef) * mecanumXbot.botVel.vel_y;
      linear_vel_y = (flow_y * 0.00001);
    }
  }
}


// 通过逆运动学得到电机转速（脉冲数）
void getMotorSpeed() {
  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  // rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}

// 串口调试输出
void debugPrint() {
  if (print_Count >= 100)  // 打印控制，控制周期1000ms
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
    u8g2.clearBuffer();                  // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB12_tf);  // choose a suitable font
    u8g2.setCursor(0, 16);
    u8g2.print("i:");
    u8g2.print(initialYaw);
    u8g2.print("n:");
    u8g2.print(newYaw);
    u8g2.setCursor(0, 32);
    u8g2.print("X:");
    u8g2.print(botOdometry.botPosition.position_x);
    u8g2.setCursor(0, 48);
    u8g2.print("Y:");
    u8g2.print(botOdometry.botPosition.position_y);
    u8g2.setCursor(0, 64);
    u8g2.print("Z:");
    u8g2.print(realYaw);
    u8g2.sendBuffer();
#endif
    print_Count = 0;
  }
}
// 伪串口1中断 更新光流数值
void serialEvent1() {
  while (FLOW_Serial.available()) {
    if (UPFLOW_STATUS_SUCCESS == LC302GS.readData(FLOW_Serial.read())) {
      flow_x = LC302GS.x_Offset;
      flow_y = LC302GS.y_Offset;

    }  // Call
  }
}

// 伪串口2中断 更新角度 相位平移
void serialEvent2() {
  lastYaw = newYawRad;  // 保存上一次的值
  while (IMU_Serial.available()) {
    JY901.CopeSerialData(IMU_Serial.read());  // Call JY901 data cope function
  }
  newYawRad = (float)JY901.stcAngle.Angle[2] / 32768 * M_PI;
  if (fabs(newYawRad - lastYaw) > M_PI)  // 如果差值大于M_PI，平移相位 每当连续相位角之间的跳跃大于π 弧度时，通过增加 ±2π 的整数倍来平移相位角，确保跳动小于 π。
  {
    if (lastYaw > 0) {
      newYawRad = newYawRad + 2 * M_PI;
    } else if (lastYaw < 0) {
      newYawRad = newYawRad - 2 * M_PI;
    }
  }
}
//abs fabs https://blog.csdn.net/liuzehn/article/details/108275485