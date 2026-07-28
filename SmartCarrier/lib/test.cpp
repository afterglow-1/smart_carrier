#include <Arduino.h>
#include "OneButton.h"                //一键启动
#include "FashionStar_UartServo.h"    // Fashion Star串口总线舵机
#include "FashionStar_SmartGripper.h" // Fashion Star智能夹具
#include "TTL_STEPPER.h"              //串口步进电机
#include "MultiStepper.h"
#include <JY901.h>
#include <HardwareTimer.h>
#include <stdio.h>
#include "AccelStepper.h"
#include "MoveByPosition.h"
#include "PID.h"
#include "MaixCam.h"

// 步进电机控制引脚
#define M1_4_EN_PIN PE13
#define M1_DIR_PIN PD6
#define M1_STP_PIN PD4
#define M2_DIR_PIN PE9
#define M2_STP_PIN PE11
#define M3_DIR_PIN PD14
#define M3_STP_PIN PD15
#define M4_DIR_PIN PC3_C
#define M4_STP_PIN PA1
#define M5_EN_PIN PE10
#define M5_DIR_PIN PE15
#define M5_STP_PIN PB11
#define motorInterfaceType 1 // Stepper Driver, 2 driver pins required

// 每走 1m 所需的脉冲数
const long PULSES_PER_METER = 10000;

// 4个电机极性参数，1 表示正常， -1 表示反转
int motorPolarity[4] = {-1, 1, -1, 1};

// 串口通信相关
#define TJCHMI_BAUDRATE 115200 // 串口屏幕波特率
#define QR_BAUDRATE 9600       // 串口扫码模块波特率

// 设备引脚定义
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14
#define QR_RX PE0
#define QR_TX PE1
#define LED_PIN PB4   // PA15 黄灯  PB4 蓝灯
#define START_BTN PB9 // V3 PB9/PB4根据实际按键引脚修改
#define TJCHMI_BAUDRATE 115200
#define WTIMU_BAUDRATE 115200
#define WTIMU_RX PD9
#define WTIMU_TX PD8

// 串口控制步进电机控制串口
#define STEPPER_TX PA2
#define STEPPER_RX PA3
#define SMALL_ARM_STEPPER_ID 6 // 悬臂轴步进电机ID
#define BIG_ARM_STEPPER_ID 7   // 丝杠升降轴步进电机ID

// 串口舵机控制串口
#define SERVO_BAUDRATE 115200 // 串口舵机波特率
#define SERVO_RX PC7
#define SERVO_TX PC6

// 舵机编号常量符号化
#define GRIPPER_SERVO_ID 4 // 手爪舵机ID号
// 储料盘编号常量符号化
#define STORAGE_SERVO_ID 5 // 储料盘舵机ID号
// 爪子的配置
#define GRIPPER_OPEN_ANGLE 32.0     // 爪子张开时的角度
#define GRIPPER_CLOSE_ANGLE 70.0    // 爪子闭合时的角度
#define GRIPPER_MAX_OPEN_ANGLE 10.0 // 爪子最大张开角度

#define CMD_SET_CIRCLE 0xEE // 指令0x02为检测加工区
// 储料盘的配置
//  载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
int storage[5] = {70, -50, 45, 135, -60};

HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
HardwareSerial Serial_QR(QR_RX, QR_TX);
// 硬件串口对象
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);
HardwareSerial Serial_Stepper(STEPPER_RX, STEPPER_TX);

// 舵机相关对象
FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE); // 协议V2版本新增
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);  // 手爪
FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);  // 储料盘
// Update the constructor parameters to match the definition in FashionStar_SmartGripper.h
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);
// 储料盘舵机初始化

// 步进电机相关对象
TTL_Protocol Stepper_protocol(&Serial_Stepper, 115200);
TTL_Stepper small_armStepper(SMALL_ARM_STEPPER_ID, &Stepper_protocol);    // 悬臂轴步进电机
TTL_Stepper big_armStepper(BIG_ARM_STEPPER_ID, &Stepper_protocol);        // 丝杠升降轴步进电机
AccelStepper rotationStepper(motorInterfaceType, M5_STP_PIN, M5_DIR_PIN); // 底部旋转轴步进电机

// 运动参数
const float PULSES_PER_REV = 200;                                                                  // 步进电机每转脉冲数
const float MICROSTEPS = 16;                                                                       // 细分倍数
const float ROTATION_GEAR_RATIO = 4;                                                               // 底部旋转轴减速比 4:1
const float ROTATION_PULSES_PER_DEG = (PULSES_PER_REV * MICROSTEPS * ROTATION_GEAR_RATIO) / 360.0; // 底部旋转轴每度脉冲数
const float LEAD_SCREW_PITCH = 12;                                                                 // 丝杠导程
const float GEAR_MODULE = 1;                                                                       // 齿轮模数
const float GEAR_TEETH = 36;                                                                       // 齿轮齿数
const float RACK_PITCH = PI * GEAR_MODULE;                                                         // 齿条齿距

// 接收数据变量
uint8_t packetBuffer[PACKET_SIZE]; // 数据包
uint8_t byteCount = 0;             // 数据长度
unsigned long lastByteTime = 0;    // 接收时间

// 全局变量
bool isStopDisplayClose = false;
// 全局变量：标记是否需要持续显示“关闭”
OneButton start_btn(START_BTN, true, true); // true:按下为低电平
bool start_flag = false;
bool scanCompleted = false;
bool isStopping = false;
char qrResult[20] = {0};
char qrResult1[20] = {0xB9, 0xD8, 0xB1, 0xD5, '\0'}; // 存储扫码结果，确保足够大
int dataIndex = 0;                                   // 数据索引
const int bufferSize = 16;                           // 增大缓冲区大小
char receivedData[bufferSize];                       // 存储接收到的数据
unsigned long lastDisplayTime = 0;                   // 上次显示时间
const unsigned long DISPLAY_INTERVAL = 1000;         // 显示更新间隔(ms)

//*********需要添加的参数************/
bool vision_updated = false;

/*定时器对象*/
// 创建一个HardwareTimer对象，选择使用TIM3，用于获取小车姿态
HardwareTimer myTimer1(TIM3);
HardwareTimer myTimer2(TIM4);
double Previous_Time, Current_Time;
int state = 0x00;
bool Ready_to_Grab = false;
double deltax, deltay;
bool flag = true;
float continuous_yaw = 0;
int circle = 0;
int i = 0;
int zhuangpei_count = 0x03;
double c_time, p_time;
int fi = 1;
int fk = 1;
double fc_time, fp_time; // 追踪相关的时间参数

// 点位结构体定义下（x ,y）
typedef struct
{
  float x;
  float y;
} CAR_GOAL_POINT_1;
CAR_GOAL_POINT_1 move_map[] =
    {
        {0.735, 0.1},  // 01 到扫码
        {0.59, -0.08}, // 02 到夹取
        {0.53, 0.05},  // 03 到转角
        {0.86, -0.05}, // 04 到粗加工
        {0.85, 0.16},  // 05 到转角
        {0.57, -0.08}, // 06 到装配
        {-0.18, 0.05}, // 07 后退
        {1.78, 0},     // 08 到夹取
        {0.15, 0},     // 09 前进
        {0, 0.16},     // 10 左走
        {0.35, 0},     // 11 到转角
        {0.85, -0.05}, // 12 到粗加工
        {0.85, 0.16},  // 13 到转角
        {0.77, 0},     // 14 到装配
        {0.75, 0.1},   // 15 到转角
};

// 有限状态机状态枚举
enum Runstate
{
  kongxian,
  saoma,
  run_move,
  rotate,
  jiaqu,
  cujiagong,
  zhuangpei,
  luzhang,
  stop
};

Runstate currentState = kongxian; // 初始状态设为空闲

const float WHEEL_BASE = 0.185;                                                       // 左右轮间距（米），例如：30cm
const float WHEEL_TRACK = 0.195;                                                      // 前后轮间距（米），例如：30cm
const float ROTATION_RADIUS = sqrt(pow(WHEEL_BASE / 2, 2) + pow(WHEEL_TRACK / 2, 2)); // 旋转半径（米）

long calculateRotatePulses(float angle)
{
  // 角度转弧度 + 计算轮子需走的弧长（米）
  float arcLength = ROTATION_RADIUS * (angle * PI / 180.0);
  // 弧长转脉冲数（脉冲/米 * 米）
  return arcLength * PULSES_PER_METER;
}

void start_click()
{
  start_flag = true;
  digitalWrite(LED_PIN, HIGH);
  Serial_TJCHMI.print("t7.txt=\"V1_ON\"\xff\xff\xff");
  // 重置电机状态
  currentState = run_move;
  scanCompleted = false;                 // 重置扫码完成标志
  memset(qrResult, 0, sizeof(qrResult)); // 清空扫码结果
}

// 显示扫码结果到屏幕
// 原displayQRResult函数
void displayQRResult()
{
  // 新增：若stop后需持续显示“关闭”，则优先显示
  if (isStopDisplayClose)
  {
    char strClose[50];
    sprintf(strClose, "t3.txt=\"%s\"\xff\xff\xff", qrResult1);
    Serial_TJCHMI.print(strClose);
    return; // 跳过原逻辑，避免被覆盖
  }

  // 原逻辑：仅在非stop状态下显示扫码结果
  if (scanCompleted && strlen(qrResult) > 0)
  {
    char strTemp[50];
    sprintf(strTemp, "t3.txt=\"%s\"\xff\xff\xff", qrResult);
    Serial_TJCHMI.print(strTemp);
    Serial_TJCHMI.print("t1.txt=\"QROK\"\xff\xff\xff");
  }
  else
  {
    Serial_TJCHMI.print("t1.txt=\"等待扫码\"\xff\xff\xff");
  }
}

float yaw;  // 当前原始角度（-180~180°）
int yaw100; // 显示用（放大100倍）
float Gyro; // 角加速度（保留原变量）
int a = 0;
unsigned long nowtime;

// 新增变量：用于角度连续性处理
float last_yaw = 0;    // 上一次角度值
int circle_count = 0;  // 累计旋转圈数（每圈360°）
bool first_run = true; // 首次运行标志

// 扫码参数
int qr_int_str[6] = {0}; // 表示存储的颜色，1R2G3B
char str[100];
bool parameter_ok = false;
bool tm0_En = false;

void testgripper()
{
  gripper.close();
  delay(1000);
  gripper.open();
  delay(1000);
}

void testmove(float distance)
{
  stepper1.move(motor1_CW * (long)((distance)*PULSES_PER_METER));
  stepper2.move(motor2_CW * (long)((distance)*PULSES_PER_METER));
  stepper3.move(motor3_CW * (long)((distance)*PULSES_PER_METER));
  stepper4.move(motor4_CW * (long)((distance)*PULSES_PER_METER));
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
  {
    RunMotors();
  }
}

void Get_Yaw()
{
  while (Serial_WTIMU.available())
  {
    JY901.CopeSerialData(Serial_WTIMU.read()); // Call JY901 data cope function
    // 储存当前的偏航角
    CurrentRad = (float)JY901.stcAngle.Angle[2] / 32768 * M_PI;
    CurrentYaw = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
  }
  // 计算角度差（关键：检测跳变）
  float delta = CurrentYaw - last_yaw;
  // 当角度从179°跳变到-179°（实际是顺时针转2°）
  if (delta < -300)
  {
    circle_count++; // 累加圈数（补偿360°）
  }
  else if (delta > 300)
  {
    circle_count--; // 递减圈数（减去360°）
  }
  // 更新上一次角度
  last_yaw = CurrentYaw;
  // 计算连续角度（原始角度 + 圈数补偿）
  continuous_yaw = CurrentYaw + circle_count * 360;
  // 转换为显示值（放大100倍，保留两位小数）
  yaw100 = (int)(continuous_yaw * 100);
  // 每100ms更新串口屏
  if (millis() >= nowtime + 1)
  {
    nowtime = millis();
    char str[100];
    sprintf(str, "x0.val=%d\xff\xff\xff", yaw100);
    Serial_TJCHMI.print(str);
    a++;
  }
}

//********需要添加函数************/
void Get_Position()
{
  while (Serial_Maix.available())
  {
    MaixCam.Maix_ReadData((uint8_t)Serial_Maix.read());
    Move_X_Grab = MaixCam.Delta_X; // 相对位移，以车为坐标系
    Move_Y_Grab = MaixCam.Delta_Y;
    Current_Color = MaixCam.Color;
    if (Current_Color == (int8_t)2 || Current_Color == (int8_t)1 || Current_Color == (int8_t)3)
    {
      vision_updated = true;
      deltax = (Move_X_Grab - 160) / 320 * 100 / 4;
      deltay = (Move_Y_Grab - 120) / 240 * 75 / 4;
      if (pow(deltax, 2) + pow(deltay, 2) < 0.01)
      {
        double k = deltay * deltax;
        deltax = (deltax > 0) ? 0.07 : -0.07;
        deltay = (k > 0) ? 0.75 * deltax : -0.75 * deltax;
      }
    }
  }
}

void setnewspeed(int speed)
{
  stepper1.setMaxSpeed(speed);
  stepper2.setMaxSpeed(speed);
  stepper3.setMaxSpeed(speed);
  stepper4.setMaxSpeed(speed);
}

void setnewaccel(int accel)
{
  stepper1.setAcceleration(accel);
  stepper2.setAcceleration(accel);
  stepper3.setAcceleration(accel);
  stepper4.setAcceleration(accel);
}

void change_car_yaw(float target_yaw, float d_angle)
{
  // 计算角度差并限制最大步数
  float angle_diff = continuous_yaw - target_yaw;
  while (fabs(angle_diff) > d_angle)
  {
    while (Serial_WTIMU.available())
    {
      JY901.CopeSerialData(Serial_WTIMU.read());
    }
    // 计算脉冲数（带比例控制）
    long pulses = calculateRotatePulses(angle_diff) * 0.5;
    if (fabs(pulses) < 0.15)
    {
      pulses = (pulses > 0) ? 0.15 : -0.15;
    }
    // 麦轮顺时针旋转逻辑：对角电机反向
    stepper1.move(pulses * motor1_CW);  // 左前：正转
    stepper2.move(-pulses * motor2_CW); // 右前：反转
    stepper3.move(pulses * motor3_CW);  // 左后：反转
    stepper4.move(-pulses * motor4_CW); // 右后：正转
    // 执行电机运动
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
    {
      RunMotors();
    }
    Get_Yaw();
    angle_diff = continuous_yaw - target_yaw;
  }
}

void change_car_speed(float speed, float target_yaw)
{
  // 计算角度差并限制最大步数
  float angle_diff = continuous_yaw - target_yaw;
  // 计算脉冲数（带比例控制）
  long d_speed = angle_diff * 100;
  // 麦轮顺时针旋转逻辑：对角电机反向
  stepper1.setSpeed((speed + d_speed) * motor1_CW); // 左前：正转
  stepper2.setSpeed((speed - d_speed) * motor2_CW); // 右前：反转
  stepper3.setSpeed((speed + d_speed) * motor3_CW); // 左后：反转
  stepper4.setSpeed((speed - d_speed) * motor4_CW); // 右后：正转
  // 执行电机运动
  RunMotors_Speed();
}

void Follow_Color(float d_delta, int d_time, float d_angle, float target_yaw, int d_delay)
{
  delay(d_delay);
  if (vision_updated)
  {
    fk = 1;
    // 抖动严重可以调大数字
    if (fabs(deltax) < d_delta && fabs(deltay) < d_delta)
    {
      Current_Time = millis();
    }
    else
    {
      Previous_Time = millis();
    }
    /**********识别准确*************/
    if (Current_Time - Previous_Time > d_time)
    {
      state = 0x22;
      vision_updated = false;
    }
    /*****移动*******/
    stepper1.move(-motor1_CW * (long)((deltax + deltay) * PulseNum_mm));
    stepper2.move(-motor2_CW * (long)((deltax - deltay) * PulseNum_mm));
    stepper3.move(-motor3_CW * (long)((deltax - deltay) * PulseNum_mm));
    stepper4.move(-motor4_CW * (long)((deltax + deltay) * PulseNum_mm));
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
    {
      RunMotors();
    }
    // 调整小车姿态
    change_car_yaw(target_yaw, d_angle);
    vision_updated = false;
  }
  else
  {
    if (fk)
    {
      fk = 0;
      fp_time = millis();
    }
    fc_time = millis();
    if (fc_time - fp_time >= 1000)
    {
      fk = 1;
      state = 0x00;
    }
  }
}

void setup()
{
  delay(1000);
  Serial_WTIMU.begin(WTIMU_BAUDRATE);
  Serial_TJCHMI.begin(TJCHMI_BAUDRATE);
  MaixCam.Maix_Init();

  delay(100);
  // 配置定时器1 为1000Hz（1ms周期）IMU的回传频率最高为1000Hz
  myTimer1.setOverflow(500, HERTZ_FORMAT);
  myTimer1.attachInterrupt(Get_Position); // 附加中断回调
  myTimer1.setInterruptPriority(1, 0);    // 设置中断优先级（可选）（抢占，响应）
  myTimer1.resume();                      // 启动定时器
  // 配置定时器2 为100Hz（10ms周期）
  myTimer2.setOverflow(100, HERTZ_FORMAT);
  myTimer2.attachInterrupt(Get_Yaw);   // 附加中断回调
  myTimer2.setInterruptPriority(2, 0); // 设置中断优先级（可选）（抢占，响应）
  myTimer2.resume();                   // 启动定时器
  while (Serial_TJCHMI.read() >= 0)
    ;
  Serial_TJCHMI.print("page main\xff\xff\xff");
  nowtime = millis();
  pinMode(LED_PIN, OUTPUT);
  pinMode(M1_4_EN_PIN, OUTPUT);
  pinMode(M5_EN_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 初始化步进电机使能引脚
  digitalWrite(M5_EN_PIN, HIGH);  // 使能步进电机 高电平失能
  digitalWrite(M1_4_EN_PIN, LOW); // 使能步进电机 低电平使能
                                  // 初始化步进电机
  rotationStepper.setMaxSpeed(24000);
  rotationStepper.setAcceleration(16000);
  Stepper_protocol.init(&Serial_Stepper, 115200);
  small_armStepper.init();
  big_armStepper.init();
  small_armStepper.set(100, 0, 0, 360, 16);
  big_armStepper.set(3000, 0, 0, 120, 16);
  // 初始化舵机
  protocol.init(&Serial_SERVO, SERVO_BAUDRATE);
  gripper.init();
  gripper.setMaxPower(700);
  gripperServo.setSpeed(500);

  delay(1000); // 等待初始化完成
  // 配置步进电机参数
  stepper1.setMaxSpeed(20000);
  stepper1.setAcceleration(10000);
  stepper2.setMaxSpeed(20000);
  stepper2.setAcceleration(10000);
  stepper3.setMaxSpeed(20000);
  stepper3.setAcceleration(10000);
  stepper4.setMaxSpeed(20000);
  stepper4.setAcceleration(10000);

  // 初始化串口
  Serial_TJCHMI.begin(TJCHMI_BAUDRATE);
  Serial_QR.begin(QR_BAUDRATE);

  // 初始化按钮
  start_btn.reset();
  start_btn.attachClick(start_click);

  // 等待设备启动
  delay(300);

  // 清空串口屏缓冲区
  while (Serial_TJCHMI.read() >= 0)
    ;

  // 初始化显示
  displayQRResult();
  delay(15000);
}

void rotateBase(float degrees)
{
  long targetPulses = degrees * ROTATION_PULSES_PER_DEG;
  rotationStepper.moveTo(targetPulses);
  while (rotationStepper.distanceToGo() != 0)
  {
    rotationStepper.run();
  }
}

// 悬臂轴移动函数
void moveSmallArm(float distance)
{
  long steps = (distance / RACK_PITCH) * PULSES_PER_REV * MICROSTEPS;
  small_armStepper.runToNewPosition(steps);
  delay(250); // 等待移动完成
}

void moveBigArm(float distance)
{
  long steps = (distance / LEAD_SCREW_PITCH) * PULSES_PER_REV * MICROSTEPS;
  big_armStepper.runToNewPosition(steps);
  delay(250); // 等待移动完成
}

void loop()
{
  // 一键启动
  start_btn.tick();
  // 定期更新显示，确保扫码结果持续显示
  if (millis() - lastDisplayTime > DISPLAY_INTERVAL)
  {
    displayQRResult();
    lastDisplayTime = millis();
  }
  // 步进电机控制逻辑
  switch (currentState)
  {
  case kongxian:
    break;
  case saoma:
  {
    rotateBase(207);
    delay(200);
    // 定期更新显示，确保扫码结果持续显示
    if (millis() - lastDisplayTime > DISPLAY_INTERVAL)
    {
      displayQRResult();
      lastDisplayTime = millis();
    }

    // 处理扫码逻辑
    while (!scanCompleted && Serial_QR.available())
    {
      char incomingByte = Serial_QR.read(); // 读取一个字节数据

      // 检查是否接收到换行符，如果是则重置
      if (incomingByte == 0x0A)
      {
        dataIndex = 0; // 重置数据索引
      }
      else
      {
        // 保存字符
        if (dataIndex < (bufferSize - 1))
        {
          receivedData[dataIndex] = incomingByte; // 将数据存储到数组中
          dataIndex++;
        }

        // 检测是否接收到回车符
        if (incomingByte == 0x0D)
        {
          receivedData[dataIndex] = '\0'; // 在数据末尾添加字符串结束符

          // 复制扫码结果到永久存储
          strncpy(qrResult, receivedData, sizeof(qrResult) - 1);

          dataIndex = 0;        // 重置数据索引
          scanCompleted = true; // 设置扫码完成标志

          // 立即更新显示
          displayQRResult();
          currentState = run_move;
        }
      }
    }
  }
  break;
  case jiaqu:
  {
    // 第一圈
    if (circle == 0)
    {
      unsigned char sendbuffer[4] = {0xAA, 0x00, 0x02, 0xBB};
      Serial_Maix.write(sendbuffer, 4);
      switch (state)
      {
      case 0x00:
        // 未检测到情况
        if (vision_updated == false)
        {
          // 调整小车位置
          stepper1.move(0.01 * PULSES_PER_METER * motorPolarity[0]);
          stepper2.move(-0.01 * PULSES_PER_METER * motorPolarity[1]);
          stepper3.move(-0.01 * PULSES_PER_METER * motorPolarity[2]);
          stepper4.move(0.01 * PULSES_PER_METER * motorPolarity[3]);
          while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
          {
            RunMotors();
          }
        }
        else
        {
          state = 0x11;
          vision_updated = false;
          setnewaccel(10000);
        }
        break;
      case 0x11:
        // 追踪颜色
        Follow_Color(1.0f, 20, 0.5f, 0, 50);
        break;
      case 0x22:
        // 夹取操作
        moveSmallArm(0.12);
        storageServo.setAngle(storage[2]); // 抓取绿色下层物料
        moveBigArm(3.55);
        delay(300);
        gripper.close();
        delay(300);
        moveBigArm(0.1);
        moveSmallArm(0);
        rotateBase(180);
        moveBigArm(1.7);
        delay(300);
        gripper.open();
        delay(300);

        moveBigArm(0.1);
        rotateBase(121);
        moveSmallArm(0.27);
        storageServo.setAngle(storage[1]); // 抓取红色下层物料
        moveBigArm(3.55);
        delay(250);
        gripper.close();
        delay(300);
        moveBigArm(0.1);
        moveSmallArm(0);
        rotateBase(180);
        moveBigArm(1.7);
        delay(250);
        gripper.open();
        delay(300);

        moveBigArm(0.1);
        rotateBase(34);
        moveSmallArm(0.29);
        storageServo.setAngle(storage[3]); // 抓取蓝色下层物料
        moveBigArm(3.6);
        delay(600);
        gripper.close();
        delay(300);
        moveBigArm(0.1);
        moveSmallArm(0);
        rotateBase(180);
        moveBigArm(1.7);
        delay(250);
        gripper.open();
        delay(300);
        moveBigArm(0.1);
        // 调整状态
        state = 0x00;
        currentState = run_move;
        setnewaccel(10000);
        break;
      }
    }
    // 第二圈，可能没用
    else
    {
      unsigned char sendbuffer[4] = {0xAA, 0x00, 0x03, 0xBB};
      Serial_Maix.write(sendbuffer, 4);
      switch (state)
      {
      case 0x00:
        // 未检测到情况
        if (vision_updated == false)
        {
          // 调整小车位置
          stepper1.move(0.01 * PULSES_PER_METER * motorPolarity[0]);
          stepper2.move(-0.01 * PULSES_PER_METER * motorPolarity[1]);
          stepper3.move(-0.01 * PULSES_PER_METER * motorPolarity[2]);
          stepper4.move(0.01 * PULSES_PER_METER * motorPolarity[3]);
          while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
          {
            RunMotors();
          }
        }
        else
        {
          gripper.openMax();
          state = 0x11;
          vision_updated = false;
          setnewaccel(10000);
        }
        break;
      case 0x11:
        // 追踪颜色
        Follow_Color(0.5f, 40, 0.5f, 364, 50);
        break;
      case 0x22:
        // 夹取操作
        moveBigArm(0.47);
        moveSmallArm(0.44);
        delay(200);
        storageServo.setAngle(storage[3]); // 抓取蓝色上层物料
        delay(250);
        gripper.close();
        delay(300);
        moveSmallArm(0);
        rotateBase(180);
        moveBigArm(1.7);
        delay(250);
        gripper.open();
        moveBigArm(0.47);
        testmove(0.16); // 移动至绿色

        rotateBase(75);
        moveSmallArm(0.42);
        delay(200);
        storageServo.setAngle(storage[2]); // 抓取绿色上层物料
        delay(250);
        gripper.close();
        delay(300);
        moveSmallArm(0);
        rotateBase(180);
        moveBigArm(1.7);
        delay(250);
        gripper.open();
        moveBigArm(0.47);
        testmove(0.16); // 移动至红色

        rotateBase(75);
        moveSmallArm(0.43);
        delay(200);
        storageServo.setAngle(storage[1]); // 抓取红色上层物料
        delay(250);
        gripper.close();
        delay(300);
        moveSmallArm(0);
        rotateBase(180);
        moveBigArm(1.7);
        delay(250);
        gripper.open();
        moveBigArm(0.4);
        // 调整状态
        state = 0x00;
        currentState = run_move;
        setnewaccel(10000);
        break;
      }
    }
  }
  break;
  case cujiagong:
  {
    unsigned char sendbuffer[4] = {0xAA, 0x00, 0x02, 0xBB};
    Serial_Maix.write(sendbuffer, 4);
    switch (state)
    {
    case 0x00:
      // 未检测到情况
      if (vision_updated == false)
      {
        // 调整小车位置
        stepper1.move(motor1_CW * (long)((0.01 + 0.01) * PULSES_PER_METER));
        stepper2.move(motor2_CW * (long)((0.01 - 0.01) * PULSES_PER_METER));
        stepper3.move(motor3_CW * (long)((0.01 - 0.01) * PULSES_PER_METER));
        stepper4.move(motor4_CW * (long)((0.01 + 0.01) * PULSES_PER_METER));
        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
        {
          RunMotors();
        }
      }
      else
      {
        state = 0x11;
        vision_updated = false;
        setnewaccel(20000);
      }
      break;
    case 0x11:
      // 追踪颜色
      Follow_Color(0.3f, 50, 0.5f, round(continuous_yaw / 90) * 90 + circle * 4.5 + 0.5, 50);
      break;
    case 0x22:
      // 夹取操作
      gripper.open();
      storageServo.setAngle(storage[3]);
      delay(1000);
      moveBigArm(0.1);
      moveSmallArm(0); // 放置蓝色物料
      rotateBase(180);
      delay(300);
      moveBigArm(1.7);
      delay(500);
      gripper.close();
      delay(1000);
      moveBigArm(0.1);
      rotateBase(32);
      moveSmallArm(0.27);
      delay(200);
      moveBigArm(1.65);
      delay(500);
      gripper.open();
      moveSmallArm(0);

      storageServo.setAngle(storage[2]);
      delay(1000); // 放置绿色物料
      moveBigArm(0.1);
      rotateBase(180);
      moveSmallArm(0);
      delay(300);
      moveBigArm(1.7);
      delay(500);
      gripper.close();
      delay(300);
      moveBigArm(0.1);
      rotateBase(76);
      moveSmallArm(0.072);
      moveBigArm(1.65);
      delay(500);
      gripper.open();
      moveSmallArm(0);

      storageServo.setAngle(storage[1]);
      delay(1000);
      moveBigArm(0.1); // 放置红色物料
      rotateBase(182);
      moveSmallArm(0);
      delay(300);
      moveBigArm(1.7);
      delay(500);
      gripper.close();

      delay(1000);
      moveBigArm(0.1);
      rotateBase(122);
      moveSmallArm(0.23);
      moveBigArm(1.65);
      delay(500);
      gripper.open();
      delay(500);
      gripper.close();
      delay(300);

      // 取回红色物料
      storageServo.setAngle(storage[3]);
      delay(1000);
      moveBigArm(0.1);
      moveSmallArm(0);
      rotateBase(180);
      moveBigArm(1.7);
      delay(300);
      gripper.open();
      delay(300);
      moveBigArm(0.1);

      // 取回绿色物料
      rotateBase(77);
      moveSmallArm(0.09);
      moveBigArm(1.7);
      delay(500);
      gripper.close();
      delay(300);
      storageServo.setAngle(storage[2]);
      moveBigArm(0.1);
      moveSmallArm(0);
      rotateBase(180);
      moveBigArm(1.7);
      delay(300);
      gripper.open();
      delay(300);
      moveBigArm(0.1);

      // 取回蓝色物料
      rotateBase(34);
      moveSmallArm(0.29);
      delay(300);
      moveBigArm(1.7);
      delay(500);
      gripper.close();
      delay(300);
      storageServo.setAngle(storage[1]);
      moveBigArm(0.1);
      moveSmallArm(0);
      rotateBase(180);
      moveBigArm(1.7);
      delay(300);
      gripper.open();
      delay(300);
      moveBigArm(0.1);
      // 调整状态
      state = 0x00;
      currentState = run_move;
      setnewaccel(10000);
      break;
    }
  }
  break;
  case zhuangpei:
  {
    if (circle == 0)
    {
      unsigned char sendbuffer[4] = {0xAA, 0x00, (unsigned char)zhuangpei_count, 0xBB};
      Serial_Maix.write(sendbuffer, 4);
      switch (state)
      {
      case 0x00:
        // 未检测到情况
        if (vision_updated == false)
        {
          // 调整小车位置
          setnewaccel(10000);
          stepper1.move(0.07 * PULSES_PER_METER * motorPolarity[0]);
          stepper2.move(0.07 * PULSES_PER_METER * motorPolarity[1]);
          stepper3.move(0.07 * PULSES_PER_METER * motorPolarity[2]);
          stepper4.move(0.07 * PULSES_PER_METER * motorPolarity[3]);
          while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
          {
            RunMotors();
          }
        }
        else
        {
          gripper.openMax();
          state = 0x11;
          vision_updated = false;
          setnewaccel(1000);
        }
        break;
      case 0x11:
        // 追踪颜色
        Follow_Color(0.1f, 100, 0.5f, 182, 100);
        gripper.open();
        break;
      case 0x22:
        // 夹取操作
        if (zhuangpei_count == 3)
        {
          storageServo.setAngle(storage[4 - (int)zhuangpei_count]);
          delay(1000);
          moveBigArm(0.1);
          moveSmallArm(0); // 放置蓝色物料
          rotateBase(180);
          delay(300);
          moveBigArm(1.7);
          delay(500);
          gripper.close();
          delay(1000);
          moveBigArm(0.1);
          rotateBase(75);
          moveSmallArm(0.075); // 放置物料
          moveBigArm(3.8);
          delay(500);
          gripper.open();
          delay(300);
          moveBigArm(0.1);
          moveSmallArm(0.168);
        }
        else if (zhuangpei_count == 2)
        {
          storageServo.setAngle(storage[4 - (int)zhuangpei_count]);
          moveBigArm(0.1);
          moveSmallArm(0); // 放置绿色物料
          rotateBase(180);
          delay(300);
          moveBigArm(1.7);
          delay(500);
          gripper.close();
          delay(1000);
          moveBigArm(0.1);
          rotateBase(75.2);
          moveSmallArm(0.0595); // 放置物料
          moveBigArm(3.8);
          delay(500);
          gripper.open();
          delay(300);
          moveBigArm(0.1);
          moveSmallArm(0.168);
        }
        else if (zhuangpei_count == 1)
        {
          storageServo.setAngle(storage[4 - (int)zhuangpei_count]);
          moveBigArm(0.1);
          moveSmallArm(0); // 放置红色物料
          rotateBase(180);
          delay(300);
          moveBigArm(1.7);
          delay(500);
          gripper.close();
          delay(1000);
          moveBigArm(0.1);
          rotateBase(75.5);
          moveSmallArm(0.0595); // 放置物料
          moveBigArm(3.8);
          delay(500);
          gripper.open();
          delay(300);
          moveBigArm(0.1);
        }
        // 调整状态
        if (zhuangpei_count == 1)
        {
          rotateBase(180);
          moveBigArm(0.1);
          moveSmallArm(0);
          currentState = run_move;
          circle++;
        }
        zhuangpei_count--;
        vision_updated = false;
        state = 0x00;
        setnewaccel(10000);
        break;
      }
    }
    else
    {
      unsigned char sendbuffer[4] = {0xAA, 0x00, 0x02, 0xBB};
      Serial_Maix.write(sendbuffer, 4);
      switch (state)
      {
      case 0x00:
        // 未检测到情况
        if (vision_updated == false)
        {
          // 调整小车位置
          stepper1.move(0.01 * PULSES_PER_METER * motorPolarity[0]);
          stepper2.move(0.01 * PULSES_PER_METER * motorPolarity[1]);
          stepper3.move(0.01 * PULSES_PER_METER * motorPolarity[2]);
          stepper4.move(0.01 * PULSES_PER_METER * motorPolarity[3]);
          while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
          {
            RunMotors();
          }
        }
        else
        {
          gripper.openMax();
          state = 0x11;
          vision_updated = false;
          setnewaccel(10000);
        }
        break;
      case 0x11:
        // 追踪颜色
        Follow_Color(0.1f, 50, 0.5f, 542, 50);
        gripper.open();
        break;
      case 0x22:
        // 夹取操作
        if (qrResult[0] == 0xCD)
        {
          storageServo.setAngle(storage[1]);
          delay(1000); // 装配蓝色物料
          moveSmallArm(0);
          moveBigArm(0.1);
          rotateBase(180);
          moveBigArm(1.7);
          delay(250);
          gripper.close();
          delay(300);
          moveBigArm(0.1);
          rotateBase(34);
          moveSmallArm(0.31);
          delay(250);
          moveBigArm(1.21);
          gripper.open();
          delay(300);

          storageServo.setAngle(storage[2]); // 装配绿色物料
          delay(1000);
          moveSmallArm(0);
          moveBigArm(0.1);
          rotateBase(180);
          moveBigArm(1.7);
          delay(250);
          gripper.close();
          delay(300);
          moveBigArm(0.1);
          rotateBase(74.5);
          moveSmallArm(0.11);
          delay(250);
          moveBigArm(1.21);
          gripper.open();
          delay(300);

          storageServo.setAngle(storage[3]); // 抓取红色下层物料
          delay(1000);
          moveSmallArm(0);
          moveBigArm(0.1);
          rotateBase(180);
          moveBigArm(1.7);
          delay(250);
          gripper.close();
          delay(300);
          moveBigArm(0.1);
          rotateBase(120);
          moveSmallArm(0.25);
          delay(250);
          moveBigArm(1.21);
          gripper.open();
          delay(300);
        }
        else
        {
          storageServo.setAngle(storage[2]);
          delay(1000); // 装配蓝色物料
          moveSmallArm(0);
          moveBigArm(0.1);
          rotateBase(180);
          moveBigArm(1.7);
          delay(250);
          gripper.close();
          delay(300);
          moveBigArm(0.1);
          rotateBase(34);
          moveSmallArm(0.31);
          delay(250);
          moveBigArm(1.21);
          gripper.open();
          delay(300);

          storageServo.setAngle(storage[3]); // 装配绿色物料
          delay(1000);
          moveSmallArm(0);
          moveBigArm(0.1);
          rotateBase(180);
          moveBigArm(1.7);
          delay(250);
          gripper.close();
          delay(300);
          moveBigArm(0.1);
          rotateBase(74.5);
          moveSmallArm(0.11);
          delay(250);
          moveBigArm(1.21);
          gripper.open();
          delay(300);

          storageServo.setAngle(storage[1]); // 装配红色下层物料
          delay(1000);
          moveSmallArm(0);
          moveBigArm(0.1);
          rotateBase(180);
          moveBigArm(1.7);
          delay(250);
          gripper.close();
          delay(300);
          moveBigArm(0.1);
          rotateBase(120);
          moveSmallArm(0.25);
          delay(250);
          moveBigArm(1.21);
          gripper.open();
          delay(300);
        }
        moveBigArm(4);
        delay(400);
        gripper.close();
        delay(200);
        moveBigArm(0.1);
        moveSmallArm(0);
        rotateBase(180);
        // 调整状态
        delay(300);
        change_car_yaw(546, 0.1f);
        state = 0x00;
        currentState = run_move;
        circle++;
        setnewaccel(10000);
        break;
      }
    }
  }
  break;
  case rotate:
  {
    // 顺时针/逆时针 90
    long pulses = calculateRotatePulses(130);
    stepper1.move(-pulses * motor1_CW); // 左前：正转
    stepper2.move(pulses * motor2_CW);  // 右前：反转
    stepper3.move(-pulses * motor3_CW); // 左后：反转
    stepper4.move(+pulses * motor4_CW); // 右后：正转
    // 执行电机运动
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
    {
      RunMotors();
    }
    change_car_yaw(round(continuous_yaw / 90) * 90 + circle + 1, 0.1f);
    currentState = run_move;
    if (i == 15)
    {
      change_car_yaw(636, 0.1f);
      currentState = luzhang;
    }
  }
  break;
  case run_move:
  {
    stepper1.move(motor1_CW * (long)((move_map[i].x - move_map[i].y) * PULSES_PER_METER));
    stepper2.move(motor2_CW * (long)((move_map[i].x + move_map[i].y) * PULSES_PER_METER));
    stepper3.move(motor3_CW * (long)((move_map[i].x + move_map[i].y) * PULSES_PER_METER));
    stepper4.move(motor4_CW * (long)((move_map[i].x - move_map[i].y) * PULSES_PER_METER));
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
    {
      RunMotors();
    }
    // i 为 9 时不用判断
    if (i == 0)
    {
      currentState = saoma;
    }
    if (i == 1)
    {
      rotateBase(75);
      moveBigArm(0.1);
      moveSmallArm(0.168);
      currentState = jiaqu;
    }
    if (i == 2 || i == 4 || i == 6 || i == 7 || i == 10 || i == 12 || i == 14)
    {
      currentState = rotate;
    }
    if (i == 3 || i == 11)
    {
      rotateBase(75);
      moveBigArm(0.3);
      moveSmallArm(0.168);
      currentState = cujiagong;
    }
    if (i == 5 || i == 13)
    {
      rotateBase(75);
      moveBigArm(1.7);
      moveSmallArm(0.168);
      currentState = zhuangpei;
    }
    if (i == 8)
    {
      rotateBase(75);
      moveSmallArm(0.05);
      currentState = jiaqu;
    }
    i++;
  }
  break;
  case luzhang:
    // 待补充
    // 电机转速设置
    if (fi)
    {
      fi = 0;
      p_time = millis();
    }
    c_time = millis();
    if (c_time - p_time >= 2650)
    {
      // 顺时针/逆时针 90
      long pulses = calculateRotatePulses(130);
      stepper1.move(-pulses * motor1_CW); // 左前：正转
      stepper2.move(pulses * motor2_CW);  // 右前：反转
      stepper3.move(-pulses * motor3_CW); // 左后：反转
      stepper4.move(+pulses * motor4_CW); // 右后：正转
      // 执行电机运动
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
      {
        RunMotors();
      }
      change_car_yaw(726, 0.1f);

      storageServo.setAngle(storage[2]);
      delay(300);
      rotateBase(191);
      moveSmallArm(0.26);
      moveBigArm(1.7);
      delay(250);
      gripper.openMax();
      delay(300);
      moveBigArm(0.2);
      delay(250);
      moveSmallArm(0);
      rotateBase(76);
      delay(250);
      storageServo.setAngle(storage[3]);
      moveBigArm(3.65);
      delay(1000);
      vision_updated = false;
      currentState = stop;
    }
    change_car_speed(6500, 636);
    break;
  case stop:
    unsigned char sendbuffer[4] = {0xAA, 0x00, 0x02, 0xBB};
    Serial_Maix.write(sendbuffer, 4);
    switch (state)
    {
    case 0x00:
      // 未检测到情况
      if (vision_updated == false)
      {
        // 调整小车位置
        setnewaccel(10000);
        stepper1.move(0.01 * PULSES_PER_METER * motorPolarity[0]);
        stepper2.move(-0.01 * PULSES_PER_METER * motorPolarity[1]);
        stepper3.move(-0.01 * PULSES_PER_METER * motorPolarity[2]);
        stepper4.move(0.01 * PULSES_PER_METER * motorPolarity[3]);
        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
        {
          RunMotors();
        }
      }
      else
      {
        gripper.openMax();
        moveBigArm(3);
        state = 0x11;
        vision_updated = false;
        setnewaccel(2000);
      }
      break;
    case 0x11:
      // 追踪颜色
      Follow_Color(10.0f, 20, 0.5f, 726, 50);
      break;
    case 0x22:
      moveBigArm(0);
      delay(250);
      vision_updated = false;
      setnewaccel(8000);
      // 回归操作
      stepper1.move(motor1_CW * (long)((-0.25 + 0.15) * PULSES_PER_METER));
      stepper2.move(motor2_CW * (long)((-0.25 - 0.15) * PULSES_PER_METER));
      stepper3.move(motor3_CW * (long)((-0.25 - 0.15) * PULSES_PER_METER));
      stepper4.move(motor4_CW * (long)((-0.25 + 0.15) * PULSES_PER_METER));
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
      {
        RunMotors();
      }
      rotateBase(2);
      delay(250);
      gripper.close();
      state = 0x33;
      break;
    case 0x33:
      isStopDisplayClose = true;
      char strTemp1[50];
      sprintf(strTemp1, "t3.txt=\"%s\"\xff\xff\xff", qrResult1);
      Serial_TJCHMI.print(strTemp1);
      // 停止操作
      state = 0x00;
      currentState = kongxian;
      break;
    }
    break;
  }
}