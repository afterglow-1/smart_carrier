//头文件引入区这个对的

#include <Arduino.h>
#include "AccelStepper.h"
#include <JY901.h>
#include "OneButton.h" 
#include "PID.h"
#include <HardwareTimer.h>
#include "FashionStar_UartServo.h"    // Fashion Star串口总线舵机
#include "FashionStar_SmartGripper.h" // Fashion Star智能夹具
#include "TTL_STEPPER.h"              //串口步进电机
#include "MaixCam.h"
#include "MultiStepper.h"
#include <stdio.h>

//define区

#define PI 3.14159265358979323846
#define M1_4_EN_PIN  PE13
#define M1_DIR_PIN  PD6
#define M1_STP_PIN  PD4
#define M2_DIR_PIN  PE9
#define M2_STP_PIN  PE11
#define M3_DIR_PIN  PD14
#define M3_STP_PIN  PD15
#define M4_DIR_PIN  PC3_C
#define M4_STP_PIN  PA1
#define M5_EN_PIN  PE10
#define M5_DIR_PIN  PE15
#define M5_STP_PIN  PB11
#define motorInterfaceType 1  // Stepper Driver, 2 driver pins required
//屏幕
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14
#define DATA_NUM 19 // 串口屏需要传输的变量个数
//扫码模块
#define QR_RX PE0
#define QR_TX PE1
//IMU
#define WTIMU_RX  PD9
#define WTIMU_TX PD8
#define START_BTN PB9     // V3 PB9/PB4根据实际按键引脚修改
//波特率
#define TJCHMI_BAUDRATE 115200  //串口屏幕波特率
#define QR_BAUDRATE 9600        //串口扫码模块波特率 默认波特率9600
#define WTIMU_BAUDRATE 115200 //串口IMU波特率
#define STEPPER_BAUDRATE 115200   // 串口步进电机波特率115200

//视觉模块参数
#define InitScale 262.0  // 摄像头初始高度下，像素个数
#define ZeroScale 20.0   // 零高度下像素个数
#define InitHeight 282.0 // 摄像头安装高度
#define ItemHeight 147   // 物料上端离地高度，单位为mm
#define ImageScale 240   // 图像y方向像素个数
#define Arm_Zero_Length 1234 // 实际上机时，机械臂抓夹中心距离转轴的距离
// 串口总线舵机配置           待修改
#define SERVO_BAUDRATE 115200     // 串口舵机波特率115200
#define SERVO_RX PC7
#define SERVO_TX PC6
#define GRIPPER_SERVO_ID 4  // 舵机4的ID号 手爪
#define STORAGE_SERVO_ID 5  // 舵机1的ID号 载物盘舵机
// 串口步进电机
// TTL串口通讯控制丝杆步进和齿轮步进
#define Stepper_TX PA2       // PB13//PA2
#define Stepper_RX PA3       // PB12//PA3//日期6/52测试PA3脚有问题
#define ARM_Stepper_ID 7     // 竖直自由度控制步进
#define Gripper_Stepper_ID 6 // 水平自由度控制步进
#define LUOGAN (12 * 10)          // 12导程
#define CHILUN (36 * 1 * PI * 10) // 36齿

//变量定义区

//任务结构体
struct Task
{
  double x;       //一次前进距离
  double y;       //二次前进距离
  double rot;     //旋转角度
  double rot2;    //二次旋转角度
  int  flag_1;    //二次定位标识
  int  flag_2;    //手爪任务标识
  float tar_rot;   //目标角度
}task[8]=
{
  {0, 530, 1,0, 0 , 0 , -1 },           // 0扫码

  {725, 0, 0 ,0, 1 , 1 , -1 },           // 1抓取

  {-390, 1680, -1,-1 , 1 , 1 , 1 }, // 2粗加工区

  {-820, -810, 1 ,0, 1 , 1,0 },  // 3存储区中心
  {-930, -470, 1 ,0, 1 , 1 , -1 },           // 4抓取

  {-400, 1680, -1,-1 , 1 , 1 , 1 }, // 5粗加工区

  {-830, -850, 1 ,0 , 1 , 1 ,0},  // 6存储区中心
  {-900, -1640, 1 , 0 , 0 , 0,0}                // 7原点
}; // 每次移动的距离，基于局部坐标

//任务枚举
enum State{
  task_0,    //一键启动
  task_0_1,  //出库
  task_1_1,    //x方向前进
  task_1_2,    //y方向前进
  task_1_3,    //旋转
  task_1_4,    //二次旋转
  task_1_5,    //任务转跳
  task_2_1,    //二次定位任务(..._1_2_3)
  task_2_2,    //二次定位任务(..._1_2_3)
  task_2_3,    //二次定位任务(..._1_2_3)
  task_2_4,    //二次定位任务(..._1_2_3)
  task_2_5,
  task_3_1,    //手爪任务跳转
  task_3_2,    //手爪任务转盘
  task_3_3,    //手爪任务粗加工
  task_3_4,    //手爪任务成品区
  task_3_5,    //手爪抓取后定位
  task_3_6,    //手爪任务跳转
};
State currentState = task_0;
//
//小车运动参数定义
int motorPolarity[4] = {-1, 1, -1, 1}; // 4个电机极性参数，1 表示正常， -1 表示反转
const long turn_Pulses = 3140;         //90°旋转角度对应的脉冲数
const long PULSES_PER_METER = 10500;   //每米对应的脉冲数
const float PULSES_PER_REV = 200;  // 步进电机每转脉冲数
const float MICROSTEPS = 16;       // 细分倍数
// 补充底部旋转轴减速比
const float ROTATION_GEAR_RATIO = 4; // 底部旋转轴减速比 4:1
// 计算底部旋转轴每度脉冲数时考虑减速比
const float ROTATION_PULSES_PER_DEG = (PULSES_PER_REV * MICROSTEPS * ROTATION_GEAR_RATIO) / 360.0;  // 底部旋转轴每度脉冲数

//手爪运动相关参数
/*高度*/
int STEPPER_ZHUANPAN = 620; // 下降到转盘抓取单位0.1毫米
int STEPPER_GROUND = 1390;  // 下降到地面
int STEPPER_STORAGE = 450;  // 下降放到载物台
int STEPPER_ZERO = 50;       // 上升回原点
int MATERIAL_HEIGHT = 680;  // 物料高度单位0.1毫米
/*机械臂伸出*/
int STEPPER_GRIPPER[4] = {20, 24, 60, 15}; // 零点，R,G,B
int STEPPER_GRIPPER_ZHUANPAN_CENTER = 60;
/*爪子*/
int GRIPPER_OPEN_ANGLE = 0;     // 爪子张开时的角度
int GRIPPER_CLOSE_ANGLE = -85;    // 爪子闭合时的角度
int GRIPPER_OPEN_MAX_ANGLE = 0; // 爪子张开最大大角度
/*基座*/    //要改成步进电机的参数（脉冲数或角度）
float ARM_BASE_STEPPER_ANGLE[4] = {15,-132, -86, -42};//机械臂底座步进运动的绝对位置；R G B Zaiwu
float ARM_BASE_STEPPER_HOME_ANGLE = 0;                // 机械臂发车初始角度        
int cishu=0;
// 载物盘舵机角度   0出发位置   1R  2G  3B  不干涉位置，储物盘三个盘位正对机械臂的角度
int storage[5] = {135, -90, 0, 95, -35};


//扫码模块相关
// 33 32 31 2B 31 32 33 0D 0A
// GM75默认是CR
// 设置后可改为CRLF
// CR（Carriage Return），回车符，用符号’\r’表示， 十进制ASCII代码是13，16进制0x0D；
// LF（Line Feed），换行符，用符号’\n’表示，十进制ASCII代码是10，16进制0x0A；
const int bufferSize = 8;       // 7字节数据 + 1字节回车符+（1字节换行符）
char receivedData[bufferSize];  // 存储接收到的数据
char strQR[] = "000+000";
bool scanFlag = false;
bool firstFlag=false;
int dataIndex = 0;  // 数据索引
int scanflag1 = 1 ;
//IMU
float yaw;   // 当前角度数据  
int yaw100;//虚拟浮点数串口屏显示用
char str[20];
float cur_rot=0;
float last_yaw;
float  accumulated_yaw;
// 视觉模块  定义三种颜色的 LAB 阈值
const uint8_t COLORS[3][4] = {
  {0xAA, 0xCC, 0x01, 0xBB},   // 红色
  {0xAA, 0xCC, 0x02, 0xBB},   // 绿色
  {0xAA, 0xCC, 0x03, 0xBB}    // 蓝色
};
unsigned char SendBuffer[4] = {0xAA, 0xEE, 0x02, 0xBB};
int8_t  dx ;
int8_t  dy ;
uint8_t colorIdx = 0;
uint32_t lastTick = 0;
const uint32_t INTERVAL = 100;   // 100 ms = 0.1 s
//其他参数
int i = 0;          //用于记录阶段
int j = 0;          //记录颜色
bool isRunning = 0; //用于判断是否运动
unsigned long nowtime;//计时
int run_flag=1;       //运动判断，避免重复设定运行脉冲
int delta_x=0;
int delta_y=0;
float MaxError_Move=2;//最大运行距离误差
float MaxError_Rot=0.1;//最大允许角度误差
float Vm;
float Scale = 0.01;//InitHeight / ImageScale; // 任意工况下测量平面单位像素的实际距离
int rounds=1;
int  task_1_4flag=0;
int time1=0;
//待调整
int qr_int_str[6] = {0}; // 表示存储的颜色，1R2G3B
bool parameter_ok = false;
bool tm0_En = false;
double Previous_Time, Current_Time;
bool Ready_to_Grab;
int Data_Index = 0;
unsigned char Data_Buffer[DATA_NUM * 4 + 6];
int num = 1;
int state = 0x00;
bool vision_updated = false;
bool gripper_state = false; // 夹爪命令状态
//实例建造


AccelStepper M1_stepper = AccelStepper(motorInterfaceType, M1_STP_PIN, M1_DIR_PIN);
AccelStepper M2_stepper = AccelStepper(motorInterfaceType, M2_STP_PIN, M2_DIR_PIN);
AccelStepper M3_stepper = AccelStepper(motorInterfaceType, M3_STP_PIN, M3_DIR_PIN);
AccelStepper M4_stepper = AccelStepper(motorInterfaceType, M4_STP_PIN, M4_DIR_PIN);
AccelStepper rotationStepper(motorInterfaceType, M5_STP_PIN, M5_DIR_PIN);  // 底部旋转轴步进电机

HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);//串口屏
HardwareSerial Serial_QR(QR_RX, QR_TX);//串口扫码模块
//HardwareSerial Serial_WTIMU(WTIMU_RX, WTIMU_TX);//串口IMU
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);//舵机
HardwareSerial Serial_Stepper(Stepper_RX, Stepper_TX);//步进电机
HardwareSerial MaixSerial(PE7, PE8);   // RX, TX 顺序 视觉模块
/**舵机***************************************************/
// 创建舵机的通信协议对象
FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE); // 协议V2版本新增
//FSUS_Servo armBaseServo(ARM_BASE_SERVO_ID, &protocol); // 机械臂舵机
FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);  // 载物盘舵机
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);  // 手爪
// 创建智能机械爪实例
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);

/**步进电机***************************************************/
// 创建步进电机的通信协议对象
TTL_Protocol Stepper_protocol(&Serial_Stepper, STEPPER_BAUDRATE);
TTL_Stepper armStepper(ARM_Stepper_ID, &Stepper_protocol);         // 机械臂竖直自由度步进，12导程，16细分步数
TTL_Stepper gripperStepper(Gripper_Stepper_ID, &Stepper_protocol); // 机械臂水平自由度电机
/*定时器对象*/
HardwareTimer myTimer1(TIM3);// 创建一个HardwareTimer对象，选择使用TIM3，用于获取小车姿态
HardwareTimer myTimer2(TIM4);// 创建一个HardwareTimer对象，选择使用TIM4，用于获取小车视觉误差
//函数声明
// 底部旋转轴旋转函数
void rotateBase(float degrees) {
  long targetPulses = degrees * ROTATION_PULSES_PER_DEG;
  rotationStepper.moveTo(targetPulses);
  while (rotationStepper.distanceToGo() != 0) {
    rotationStepper.run();
  }
}

void rotateRelativeBase(float degree){
  long movePulses = degree * ROTATION_PULSES_PER_DEG;
  rotationStepper.move(movePulses);
   while (rotationStepper.distanceToGo() != 0){
    rotationStepper.run();
   }
}

// 扫码获得字符串转换类型
void return_qr_int()
{
    int i = 0;
    for (i = 0; i < 3; i++)
    {
        switch (receivedData[i])
        { // qr_int_str 123
        case '1':
            qr_int_str[i] = 1;
            break;
        case '2':
            qr_int_str[i] = 2;
            break;
        case '3':
            qr_int_str[i] = 3;
            break;
        }
    }
    for (i = 4; i < 7; i++)
    {
        switch (receivedData[i])
        { // qr_int_str 456
        case '1':
            qr_int_str[i - 1] = 1;
            break;
        case '2':
            qr_int_str[i - 1] = 2;
            break;
        case '3':
            qr_int_str[i - 1] = 3;
            break;
        }
    }
}
//一键启动
OneButton start_btn(START_BTN, true, true);  // true:按下为低电平
bool start_flag = 0;
void start_click() {
  start_flag = 1;
  // digitalWrite(EN_PIN, LOW);
  // save_flag=1;
}
//置零position
void refresh(){
  MaixCam.Delta_X=0;
  MaixCam.Delta_Y=0; 
}
// 用于电机的复位操作
void Motor_Init() 
{
    M1_stepper.setCurrentPosition(0); // 复位步进电机初始位置
    M2_stepper.setCurrentPosition(0);
    M3_stepper.setCurrentPosition(0);
    M4_stepper.setCurrentPosition(0);
}
//设定运动的转速与加速时间
void Motor_Setup(float Vm, float Accel_Time) // Vm为转速,Accel_Time为到达最大速度的时间，单位为s
{
    // 用于设置电机的转速、加速度等
    float MaxSpeed = Vm*PULSES_PER_METER;
    float Acceleration = Vm*PULSES_PER_METER/Accel_Time;
    // 电机初始化操作
    M1_stepper.setMaxSpeed(MaxSpeed); // 设置1#电机最大速度，单位为脉冲数/s；
    M1_stepper.setAcceleration(Acceleration);

    M2_stepper.setMaxSpeed(MaxSpeed); // 设置2#电机最大速度，单位为脉冲数/s；
    M2_stepper.setAcceleration(Acceleration);

    M4_stepper.setMaxSpeed(MaxSpeed); // 设置3#电机最大速度，单位为脉冲数/s；
    M4_stepper.setAcceleration(Acceleration);

    M3_stepper.setMaxSpeed(MaxSpeed); // 设置4#电机最大速度，单位为脉冲数/s；
    M3_stepper.setAcceleration(Acceleration);
}
//电机运转
void RunMotors()
{
    // 电机运转函数，发送脉冲
    M1_stepper.run();
    M2_stepper.run();
    M3_stepper.run();
    M4_stepper.run();
}

int IsRunning()//判断是否运动
{
  isRunning = (M1_stepper.distanceToGo() == 0 && M2_stepper.distanceToGo() == 0 &&M3_stepper.distanceToGo() == 0 && M4_stepper.distanceToGo() == 0);
  isRunning=1-isRunning;
  return isRunning;
}

void Get_cur_rot(){//返回当前角度并显示
  JY901.CopeSerialData(Serial_WTIMU.read());
  yaw = (float)JY901.stcAngle.Angle[2]/32768*180;
  yaw100=(int)(yaw*100);
  //用sprintf来格式化字符串，给x0的val属性赋值
  sprintf(str, "x0.val=%d\xff\xff\xff",  yaw100);
  //把字符串发送出去
  Serial_TJCHMI.print(str);
  cur_rot= yaw/90;
}
// 收起机械臂、载物台
void Arm_Init()
{
    armStepper.runToNewPosition(10);
    armStepper.wait();
    gripperStepper.runToNewPosition(10);
    gripperStepper.wait();
    gripper.close();                                  // 爪子闭合
    //storageServo.setAngle(storage[0]);                // 载物盘归位
    //armBaseServo.setAngle(ARM_BASE_SERVO_HOME_ANGLE); // 机械臂归位（改成步进电机）
    rotateBase(0);//旋转轴归0 
    gripper.wait();
    //storageServo.wait();
    //armBaseServo.wait();//改成步进的wait
};
//task1相关函数（小车开环前进以及小车PID旋转控制）
void Run_forward(double distance){                                       //开环前进,输入参数为前进距离，进一步考虑实时分配转速提高速度
  if(fabs(distance)<=100){
    Vm=distance/70;
  }else if(fabs(distance)<1000){
    Vm=distance/500;
  }else{
    Vm=distance/1000;
  }
  Motor_Setup(Vm,1);
  M1_stepper.move(distance /1000*PULSES_PER_METER* motorPolarity[0]);
  M2_stepper.move(distance /1000*PULSES_PER_METER* motorPolarity[1]);
  M3_stepper.move(distance /1000*PULSES_PER_METER* motorPolarity[2]);
  M4_stepper.move(distance /1000*PULSES_PER_METER* motorPolarity[3]);
  RunMotors();
}

void RotateCar(float rot,float adaptiveRpm ,float Accel_Time)
{
    // 原地自转角度，只需要简化成为每次旋转M_PI/2即可，并指定方向，以俯视逆时针为正
    Motor_Setup(adaptiveRpm, Accel_Time);

    M1_stepper.move(motorPolarity[0] * (long)(rot * turn_Pulses));
    M2_stepper.move(motorPolarity[1] * (long)(-rot * turn_Pulses));
    M3_stepper.move(motorPolarity[2] * (long)(rot * turn_Pulses));
    M4_stepper.move(motorPolarity[3] * (long)(-rot * turn_Pulses));
    RunMotors();
}
/// @brief 用于全局坐标下的闭环角度调整
void RotateCar_toTarget(float TargetRad, float Rpm, float Accel_Time)
{   
   // cur_rot=0;//Get_cur_rot();
   /*// 用于原地自转，函数参数为当前姿态角与目标姿态角，转速以及加速度，加入PID反馈，适用于绝对坐标下的走点
    Rot_PID.PID_Calc(TargetRad, cur_rot); // 更新误差以及输出量
    // 动态调整速度：大误差时高速，小误差时低速
    float adaptiveRpm = Rpm;
    float SlowDown_rad = 0.1;

    if (fabs(Rot_PID.error) < SlowDown_rad) // 0.1rad ≈ 6°
    {
        adaptiveRpm = fabs(Rpm * (Rot_PID.error / (SlowDown_rad))); // 线性减速
    }
    RotateCar(Rot_PID.output, adaptiveRpm, Accel_Time);*/ 
    Motor_Setup(Rpm, Accel_Time);
    M1_stepper.move(motorPolarity[0] * (long)(TargetRad * turn_Pulses));
    M2_stepper.move(motorPolarity[1] * (long)(-TargetRad * turn_Pulses));
    M3_stepper.move(motorPolarity[2] * (long)(TargetRad * turn_Pulses));
    M4_stepper.move(motorPolarity[3] * (long)(-TargetRad * turn_Pulses));
    RunMotors();    
}

//task2
// 更新视觉比例系数
void Update_Scale(float CurrentHeight, float TargetHeight)
{
    float CameraHeight = InitHeight - CurrentHeight;
    float DeltaHeight = CameraHeight - TargetHeight;
    Scale = ((InitScale - ZeroScale) / InitHeight * DeltaHeight + ZeroScale) / ImageScale; // 当前工况下
}
//通过视觉模块得到距离误差
void Get_position_error()
{
      while (MaixSerial.available() > 0 ) {
        uint8_t data = MaixSerial.read();
        MaixCam.Maix_ReadData(data); // 实时处理每个字节
        if (MaixCam.byteCount == 0) {
          vision_updated = true; // 标记视觉数据已更新
          break;
        }
}
}
//向目标位置移动
void Run_PID_forward(float x){
  Vm=fabs(x)/100;
  Motor_Setup(Vm,1);
  if(i==1||i==4){
    x=0.5*x;
  }else if(i==6){
    x=0.4*x;
  }else{
    x=0.6*x;
  }
  M1_stepper.move(x /1000*PULSES_PER_METER* motorPolarity[0]);
  M2_stepper.move(x /1000*PULSES_PER_METER* motorPolarity[1]);
  M3_stepper.move(x /1000*PULSES_PER_METER* motorPolarity[2]);
  M4_stepper.move(x /1000*PULSES_PER_METER* motorPolarity[3]);
  RunMotors();  
}

void Run_PID_left(float y){
  Vm=fabs(y)/100;
  Motor_Setup(Vm,1);
  if(i==1||i==4){
    y=0.5*y;
  }else if(i==6){
    y=0.5*y;
  }
  else{
    y=0.6*y;
  }
  M1_stepper.move(-y /1000*PULSES_PER_METER* motorPolarity[0]);
  M2_stepper.move(y /1000*PULSES_PER_METER* motorPolarity[1]);
  M3_stepper.move(y /1000*PULSES_PER_METER* motorPolarity[2]);
  M4_stepper.move(-y /1000*PULSES_PER_METER* motorPolarity[3]);
  RunMotors();  
}

//task3

// 从转盘抓取color物料，并放置在载物台上，最终机械臂朝载物台
void Grab_ZhuanPan_to_Storage(int color)
{
    time1=0;
    gripperServo.setAngle(0);
    rotateBase(-89);// armBaseServo.setAngle(ARM_BASE_SERVO_ANGLE[2]); // 机械臂朝外
    storageServo.setAngle(storage[color]); // 载物盘旋转
    //gripper.openMax();                              // 机械爪打开
    //rotateBase(-45);//机械臂朝外并等待到位
    //armBaseServo.wait();                            // 等待机械臂旋转到位（改步进电机）
    armStepper.runToNewPosition(0);  
    gripperStepper.runToNewPosition(0);
   // armStepper.wait();                              // 等待机械臂下降到位
   // gripper.close();                                // 机械抓夹紧
    //gripper.wait();                                 // 夹紧到位
    do{
      MaixSerial.write(COLORS[color-1],4);
      MaixSerial.flush(0);
      while (!vision_updated)
      {
        ;
      }
      vision_updated = false;
      delay(100);
      if(MaixCam.Delta_Y>150){
        delay(1000);
      }
      if(MaixCam.Delta_X>80&&MaixCam.Delta_Y<20&&MaixCam.Delta_X!=160&&MaixCam.Delta_Y!=-120){
        //rotateBase(-69);
        //rotateRelativeBase(20);
        Run_forward(100);
        isRunning=IsRunning();
        while(isRunning){
          M1_stepper.run();
          M2_stepper.run();
          M3_stepper.run();
          M4_stepper.run();
          isRunning=IsRunning();
        }
        cishu++;
      }else if(MaixCam.Delta_X<-80&&MaixCam.Delta_Y<20){
        //rotateBase(-110);
        //rotateRelativeBase(-20);
        Run_forward(-80);
        isRunning=IsRunning();
        while(isRunning){
          M1_stepper.run();
          M2_stepper.run();
          M3_stepper.run();
          M4_stepper.run();
          isRunning=IsRunning();
        }
        cishu--;
      }else{
        ;
      }
      if(cishu==2||cishu==-2||time1==50){
        if(cishu>0){
          Run_forward(-cishu*100);
        }else{
          Run_forward(-cishu*80);
        }
        isRunning=IsRunning();
        while(isRunning){
          M1_stepper.run();
          M2_stepper.run();
          M3_stepper.run();
          M4_stepper.run();
          isRunning=IsRunning();
        }
        cishu=0;
        time1=0;
      }
      delay(100);
      MaixSerial.write(COLORS[color-1],4);
      MaixSerial.flush(0);
      time1++;

    }while((MaixCam.Delta_X>80||MaixCam.Delta_X<-80)||(MaixCam.Delta_Y>80||MaixCam.Delta_Y<-80)||cishu==0);
    MaixSerial.write(COLORS[color-1],4);
    MaixSerial.flush(0);
    while (!vision_updated)
    {
      ;
    }
    vision_updated = false;
    if(MaixCam.Delta_Y<-20){
        Run_PID_left(MaixCam.Delta_Y*0.6);
        isRunning=IsRunning();
        while(isRunning){
          M1_stepper.run();
          M2_stepper.run();
          M3_stepper.run();
          M4_stepper.run();
          isRunning=IsRunning();
        }
    }
    gripperStepper.runToNewPosition(0);
    armStepper.runToNewPosition(STEPPER_ZHUANPAN+400);
    /*if(MaixCam.Delta_Y<0){
      gripperStepper.runToNewPosition(0);
      armStepper.runToNewPosition(STEPPER_ZHUANPAN);
    }else{
      gripperStepper.runToNewPosition(80);
      armStepper.runToNewPosition(STEPPER_ZHUANPAN);     
    }*/
    delay(200);
    gripperServo.setAngle(-85);
    //delay(500);
    gripperServo.wait();
    //gripperStepper.runToNewPosition(5);
    armStepper.runToNewPosition(STEPPER_ZERO);     // 机械臂上升至最高位置
    //gripperStepper.runToNewPosition(STEPPER_ZERO); // 机械臂收回减少转动惯量
    delay(100);                         // 上升到位
    //delay(500);
    rotateBase(15);
    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);
    delay(150);
    //armBaseServo.setAngle(ARM_BASE_SERVO_ANGLE[0]); // 机械臂朝向载物盘（改步进电机）
    //gripper.wait();                          // 机械臂等待收回到位
    //armBaseServo.wait();                            // 等待机械臂旋转到位（改为步进电机ID:5）
    //storageServo.wait();                            // 等待载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE+400);  
    if(cishu>0){
      Run_forward(-cishu*100);
    }else{
      Run_forward(-cishu*80);
    }
    isRunning=IsRunning();
    while(isRunning){
      M1_stepper.run();
      M2_stepper.run();
      M3_stepper.run();
      M4_stepper.run();
      isRunning=IsRunning();
    }
    cishu=0;
   // delay(300);
    //rotateBase(15);
   // armStepper.runToNewPosition(STEPPER_STORAGE);   // 机械臂下降到载物台
   // armStepper.wait();                              // 等待下降到位
    //gripper.open();                                 // 夹爪打开，内置到位等待
    gripperServo.setAngle(-35);
    delay(100);
    armStepper.runToNewPosition(STEPPER_ZERO);      // 机械臂上升到最高位置
    delay(200);

   // armStepper.wait();                              // 上升到位
}
// 从载物台抓取color物料，并放置在地上/堆垛物料上表面，最终机械臂朝向载物台
void Grab_Storage_to_Place(int n)
{
    armStepper.runToNewPosition(STEPPER_ZERO);
   gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);//夹爪到载物盘
    storageServo.setAngle(storage[qr_int_str[dataIndex]]); // 载物盘旋转
    gripperServo.setAngle(-35);                                       // 爪子打开
    delay(300);
   // gripperServo.wait();
    //armBaseServo.wait();                                   // 机械臂旋转到位（改步进）
    rotateBase(15);
    delay(300);
   // storageServo.wait();                                   // 载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE+n*400);          // 机械臂下降到载物台
   // armStepper.wait();                                     // 等待下降到位
    delay(300);
    
    gripperServo.setAngle(-85);                                      // 夹爪关闭
    delay(300);
   // gripper.wait();                                        // 夹爪夹紧到位
    armStepper.runToNewPosition(STEPPER_ZERO);             // 机械臂上升
    delay(300);
    //storageServo.setAngle(storage[1]); // 载物盘旋转防止干涉
    //armStepper.wait();                                     // 上升到位
   /* if (qr_int_str[dataIndex] == 2)
    {
        storageServo.setAngle(storage[4]); // 载物台旋转朝外，避免齿条干涉***********************
    }
    else
    {
        if (dataIndex != (2 + rounds * 3))  //和圈数的关系？
        {
            storageServo.setAngle(storage[qr_int_str[dataIndex + 1]]);
        }
        else
        {
            storageServo.setAngle(storage[qr_int_str[rounds * 3]]);
        }
    }//虽然不明白，但这一串是在控制储物盘转动*/

    //armBaseServo.setAngle(ARM_BASE_SERVO_ANGLE[qr_int_str[dataIndex]]); // 旋转到对应物料放置角度（改成步进电机）
    rotateBase(ARM_BASE_STEPPER_ANGLE[qr_int_str[dataIndex]]);
    /*if (qr_int_str[dataIndex] == 2)
    {
        storageServo.wait(); // 载物台等待*****************************************
    }*/
  //armBaseServo.wait();                                                     // 机械臂旋转到位（步进）
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[qr_int_str[dataIndex]]); // 夹爪伸出对应长度
    armStepper.runToNewPosition(STEPPER_GROUND - n * MATERIAL_HEIGHT+n*400);       // 机械臂下降到地面/堆垛物料上表面
    //gripperStepper.wait();                                                   // 夹爪伸出到位
    //armStepper.wait();                                                       // 机械臂下降到位
    delay(1000);
    gripperServo.setAngle(0);                                                          // 夹爪打开，内置到位等待
    delay(300);
    if (dataIndex == (2 + rounds * 3))
    {
        armStepper.runToNewPosition(STEPPER_ZHUANPAN);
        delay(500);
        gripperStepper.runToNewPosition(STEPPER_GRIPPER[0]);
       // gripperStepper.wait();
       // armStepper.wait();
    }
    else
    {
        armStepper.runToNewPosition(STEPPER_ZERO);           // 机械臂上升
        delay(500);
        gripperStepper.runToNewPosition(70);    // 夹爪收回
       // armStepper.wait();                                   // 机械臂上升到位
       // gripperStepper.wait();                                  // 夹爪收回到位
        //armBaseServo.setAngle(ARM_BASE_SERVO_ANGLE[0]);      // 机械臂朝向载物台(步进)
        rotateBase(ARM_BASE_STEPPER_ANGLE[0]);
        //armBaseServo.wait();                                 // 机械臂旋转到位(步进)
    }
}
// 从地上抓取color物料放置到载物盘
void Grab_Ground_to_Storage()
{
   /* if (qr_int_str[dataIndex] == 2)
    {
        storageServo.setAngle(storage[4]); // 载物台旋转朝外，避免齿条干涉***********************
    }
    else
    {
        storageServo.setAngle(storage[qr_int_str[dataIndex]]);
    }*/
    //armBaseServo.setAngle(ARM_BASE_SERVO_ANGLE[qr_int_str[dataIndex]]); // 机械臂朝外(步进)
    //armBaseServo.wait();
    storageServo.setAngle(storage[1]); // 载物盘旋转防止干涉
    rotateBase(ARM_BASE_STEPPER_ANGLE[qr_int_str[dataIndex]]);    // 等待机械臂旋转到位(步进)
    gripperServo.setAngle(-35);                                                  // 机械爪打开
    /*if (qr_int_str[dataIndex] == 2)
    {
       // storageServo.wait(); // 等待载物盘旋转到位*******************
    }*/
    gripperStepper.runToNewPosition(STEPPER_GRIPPER[qr_int_str[dataIndex]]); // 夹爪伸出对应长度
    armStepper.runToNewPosition(STEPPER_GROUND); 
    delay(400);
    storageServo.setAngle(storage[qr_int_str[dataIndex]]);  
    delay(300);                // 机械臂下降到地面
   // gripper.wait();                                                          // 张开到位
  //  gripperStepper.wait();                                                   // 等待夹爪伸出到位
   // armStepper.wait();                                                       // 等待机械臂下降到位
    gripperServo.setAngle(-85);                                                        // 机械抓夹紧
   //gripper.wait();                                                          // 夹爪到位
   delay(500);
    armStepper.runToNewPosition(STEPPER_ZERO);                               // 机械臂上升至最高位置
    gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);                     // 夹爪收回
   // armStepper.wait();                                                       // 上升到位
   // gripperStepper.wait();                                                   // 夹爪收回到位
    delay(500);
    if (qr_int_str[dataIndex] == 2)
    {
      //  storageServo.setAngle(storage[qr_int_str[dataIndex]]);
    }
    //armBaseServo.setAngle(ARM_BASE_SERVO_ANGLE[0]); // 机械臂朝向载物台(步进)
    //armBaseServo.wait();                            // 机械臂旋转到位 (步进)
    rotateBase(ARM_BASE_STEPPER_ANGLE[0]);    // 等待机械臂旋转到位(步进)
    delay(200);
   // storageServo.wait();                            // 载物台旋转到位
    armStepper.runToNewPosition(STEPPER_STORAGE);   // 机械臂下降
    delay(200);
   // armStepper.wait();                              // 机械臂下降到位
    gripperServo.setAngle(-35);                                 // 夹爪打开，自带到位
    delay(500);
    armStepper.runToNewPosition(STEPPER_ZERO);      // 机械臂上升到最高点
   // armStepper.wait();                              // 等待上升到位
}
// 转盘处伸出长度
void gripper_reachout(float delta)
{
    float absolute = delta + gripperStepper.Calculate_CurrentPos();
    float x = (absolute > 1500) ? 1500 : ((absolute < 0) ? 0 : absolute);
    gripperStepper.runToNewPosition(x);
}







//主函数
void setup(){
  Serial_TJCHMI.begin(TJCHMI_BAUDRATE); //串口屏串口初始化
  sprintf(str, "rest\xff\xff\xff");
  Serial_TJCHMI.print(str); // 串口屏重启
  //串口屏串口清除缓存
  //因为串口屏开机会发送88 ff ff ff,所以要清空串口缓冲区
  while (Serial_TJCHMI.read() >= 0){
  Serial_TJCHMI.print("page main\xff\xff\xff");  //发送命令让屏幕跳转到main页面
  }
  start_btn.reset();                    // 清除一下按钮状态机的状态
  start_btn.attachClick(start_click);   // 绑定按钮点击事件
 //模块初始化
  Motor_Init(); 
  pinMode(M5_EN_PIN, OUTPUT);
  pinMode(M1_4_EN_PIN, OUTPUT);
  // IMU初始化
  IMU_Init();
  delay(100);                        // 电机复位操作
  Serial_QR.begin(QR_BAUDRATE);         //扫码串口初始化
 // Serial.begin(115200);
  //MaixSerial.begin(115200);
  MaixSerial.begin(115200); // 初始化 MaixCam 串口
  MaixSerial.write(SendBuffer, sizeof(SendBuffer));
  MaixSerial.flush();
  //while (!Serial && !MaixSerial);
 // Serial.println("STM32 -> auto color sequence 0.1 s started.");
  Serial_WTIMU.begin(WTIMU_BAUDRATE);   //IMU串口初始化
 
 /* // 配置定时器为1000Hz（1ms周期）IMU的回传频率最高为1000Hz
  myTimer1.setOverflow(10, HERTZ_FORMAT);
  myTimer1.attachInterrupt(Get_cur_rot); // 附加中断回调
  myTimer1.setInterruptPriority(1, 1);    // 设置中断优先级（可选）（抢占，响应）
  myTimer1.resume();*/
    // 配置定时器为1000Hz（1ms周期）视觉模块的回传频率最高为1000Hz
  myTimer2.setOverflow(100, HERTZ_FORMAT);
  myTimer2.attachInterrupt(Get_position_error); // 附加中断回调
  myTimer2.setInterruptPriority(1, 0);    // 设置中断优先级（可选）（抢占，响应）
  myTimer2.resume();
  digitalWrite(M5_EN_PIN, LOW);        // 使能步进电机 高电平失能
  digitalWrite(M1_4_EN_PIN, LOW);       // 使能步进电机 低电平使能
  delay(300);



  // 串口步进初始化
  Stepper_protocol.init(&Serial_Stepper, STEPPER_BAUDRATE); // 电机通信协议初始
  armStepper.init();
  gripperStepper.init();
 // gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);

  armStepper.set(5000, 0, 0, LUOGAN, 16);
  gripperStepper.set(1000, 1000, 1, CHILUN, 256);
  gripperStepper.runToNewPosition(STEPPER_GRIPPER_ZHUANPAN_CENTER);                     // 夹爪收回
  rotationStepper.setMaxSpeed(60000);  // 设置底部旋转轴最大速度
  rotationStepper.setAcceleration(40000); // 设置底部旋转轴加速度
  //舵机初始化
  protocol.init(&Serial_SERVO, SERVO_BAUDRATE); // 舵机通信协议初始化
  //armBaseServo.init();                          // 机械臂旋转基座舵机初始化
  storageServo.init();                          // 储物盘舵机初始化
  //gripper.init();                               // 手爪舵机初始化，原始程序爪子会开启
  gripperServo.init();
  storageServo.setAngleRange(-180,180);
  //gripper.setMaxPower(700);                     // 设置最大功率，单位mW
  //armBaseServo.setSpeed(500);                   // 舵机0初始化速度 机械臂舵机
  gripperServo.setAngle(-35);
  storageServo.setSpeed(300);                   // 舵机1初始化速度 储物盘
  gripperServo.setSpeed(500);
  
  //Arm_Init(); // 舵机、步进收起  


  // PID初始化设置
  Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut);
  nowtime = millis();                   // 获取当前已经运行的时间
}


void loop(){
  start_btn.tick();
  /***  hwt101只能输出z轴角度，角加速度  ***/
  // 串口接收到数据后，进行数据的读取与存储。
   if (Serial_WTIMU.available()) {
        JY901.CopeSerialData(Serial_WTIMU.read()); // Call JY901 data cope function
    }
    
    // 储存数据 角度值
    yaw = (float)JY901.stcAngle.Angle[2]/32768*180;
    
    /*// 计算角度差值
    float delta_yaw = yaw - last_yaw;
    // 处理角度突变
    if (delta_yaw > 180) {
        delta_yaw -= 360;
    } else if (delta_yaw < -180) {
        delta_yaw += 360;
    }
    // 累积角度
    accumulated_yaw += delta_yaw;
    last_yaw = yaw;*/
    cur_rot=yaw/90;
    yaw100 = (int)(yaw * 100);

    // 每100ms更新一次串口屏
    if (millis() >= nowtime + 100) {
        char str[100];
        nowtime = millis(); // 获取当前已经运行的时间
        // 用sprintf来格式化字符串，给x0的val属性赋值
        sprintf(str, "x0.val=%d\xff\xff\xff",  yaw100);
        // 把字符串发送出去
        Serial_TJCHMI.print(str);
    } 
  // ② 实时接收 MaixCam 返回的偏差
 /* if (MaixSerial.available() >= 4)
  {
    uint8_t head = MaixSerial.read();
    int8_t  dx   = MaixSerial.read();
    int8_t  dy   = MaixSerial.read();
    uint8_t tail = MaixSerial.read();

    if (head == 0xAA && tail == 0xBB)
    {
      Serial.print("Color: ");
      Serial.print(colorIdx == 0 ? 3 : (colorIdx - 1)); // 当前颜色
      Serial.print("  ΔX: ");
      Serial.print(dx);
      Serial.print("  ΔY: ");
      Serial.println(dy);
    }
  }*/
  if (scanFlag == false && scanflag1) {
    while (Serial_QR.available()) {
      char incomingByte = Serial_QR.read();  // 读取一个字节数据
      // 检查是否接收到换行符，如果是换行符则重新开始
      if (incomingByte == 0x0A) {
        dataIndex = 0;  // 重置数据索引
      } 
      else {// 保存字符
        if (dataIndex < (bufferSize - 1)) {
          receivedData[dataIndex] = incomingByte;  // 将数据存储到数组中
          dataIndex++;
        }
        if (incomingByte == 0x0D) {
          receivedData[dataIndex] = '\0';  // 在数据末尾添加字符串结束符
          dataIndex = 0;                   // 重置数据索引
          scanFlag = true;                 //数据接收成功
                                          // 处理接收到的数据，可以在这里添加你的处理逻辑
        //strcpy(strQR, receivedData);
        }
      }
    }
  }
  if(scanFlag&&scanflag1)
  {
    return_qr_int(); // 获取任务顺序
    Serial_TJCHMI.print("t1.txt=\"QROK\"\xff\xff\xff");
    //刷新屏幕显示
    char strTemp[20];
    sprintf(strTemp, "t3.txt=\"%s\"\xff\xff\xff", receivedData);
    //把字符串发送出去
    Serial_TJCHMI.print(strTemp);
    scanflag1=0;
  }
  switch (currentState)
  {
  case task_0:
    if (start_flag)
    {
      currentState = task_0_1;
    }
    break;
  case task_0_1:
  if(run_flag){
    Motor_Setup(300,2);
    M1_stepper.move((float)200 /1000*PULSES_PER_METER* motorPolarity[0]);
    M2_stepper.move(0);
    M3_stepper.move(0);
    M4_stepper.move((float)200 /1000*PULSES_PER_METER* motorPolarity[3]);
    if(i>5){
      M1_stepper.move((float)-400 /1000*PULSES_PER_METER* motorPolarity[0]);
      M2_stepper.move(0);
      M3_stepper.move(0);
      M4_stepper.move((float)-400 /1000*PULSES_PER_METER* motorPolarity[3]);
    }
    RunMotors();
    run_flag=0;
  }
  isRunning = IsRunning();
  if(!isRunning)
  {
    run_flag=1;
    currentState = task_1_1;
  }
  break;
  case task_1_1:
    if(run_flag)
    {
      Run_forward(task[i].x);
     // delay(500);
      run_flag=0;
    }

    isRunning = IsRunning();
    if(!isRunning)
    {
      run_flag=1;
      currentState = task_1_2;
    }
    break;
  case task_1_2:
    if(run_flag)
    {
      RotateCar_toTarget(task[i].rot, 10, 10);
      run_flag=0;
    }
    isRunning = IsRunning();
    if(!isRunning) //&& fabs(task[i].rot-cur_rot) <= MaxError_Rot)
    {
      run_flag=1;
      currentState = task_1_3;
    }else /*if*/{//!isRunning //&& fabs(task[i].rot-cur_rot) > MaxError_Rot){
      run_flag=0;
    }/*else{
      ;
    }*/
    break;
  case task_1_3:
    if(run_flag)
    {
     // delay(500);
    if(i==1){
      Run_PID_left(-135);
    }else{
      Run_forward(task[i].y);
    }
      run_flag=0;
    }
    isRunning = IsRunning();
    if(!isRunning )       //如果有二次定位任务则进入task_2,无则进入第二阶段i++。
    {
      run_flag=1;
      currentState = task_1_4;
    }
    break;
  case task_1_4:
    if(run_flag&&task[i].rot2!=0)
    {
      RotateCar_toTarget(task[i].rot2, 20, 10);
      run_flag=0;
    }else{
      task_1_4flag=1;
    }
    isRunning = IsRunning();
    if(!isRunning /*&& fabs(Rot_PID.error) <= MaxError_Rot*/||task_1_4flag)
    {
      run_flag=1;
      task_1_4flag=0;
      currentState = task_1_5;
    }
    break;
  case task_1_5:
   // cur_rot=Get_cur_rot();
    if(/*fabs(task[i].tar_rot-cur_rot)>MaxError_Rot&&*/run_flag){
    RotateCar_toTarget(cur_rot-task[i].tar_rot, 1, 1);
    if(i==0){
      delay(200);
    }
      run_flag=0; 
    }
    if(i==4&&rounds==1){
      rounds++;
    }
    isRunning = IsRunning();

    if(task[i].flag_1 == 0 && !isRunning)       //如果有二次定位任务则进入task_2,无则进入第二阶段i++。
    {
      i++;
      run_flag=1;
      currentState = task_1_1;
      if(i==8){
        currentState = task_0_1;
      }
    }
    else if(task[i].flag_1 == 1 && !isRunning)
    {
      run_flag=1;
      currentState = task_2_1;
    }else{
      ;
    }
    break;
  case task_2_1://手爪就位
   //Update_Scale(STEPPER_ZERO / 10, 0);
  MaixSerial.write(SendBuffer, sizeof(SendBuffer));
  MaixSerial.flush();
  storageServo.setAngle(storage[1]);
  rotateBase(-89); 
    //armBaseServo.setAngle(ARM_BASE_SERVO_ANGLE[2]); // 机械臂朝外
    //storageServo.setAngle(storage[4]);              // 载物台旋转
   // gripper.openMax();
   // gripperStepper.wait();
  // delay(1000);
    //armBaseServo.wait();
  if(i==1||i==4){
    gripperStepper.runToNewPosition(10);    
	  armStepper.runToNewPosition(0);   // 机械臂下降
    delay(500);
    currentState = task_3_1;
  }else{
    gripperStepper.runToNewPosition(55);
	  armStepper.runToNewPosition(1400);   // 机械臂下降
    delay(500);
    currentState = task_2_4;
  }
  
    break;
  case task_2_2:
    if(run_flag)
    { 
      MaixSerial.write(SendBuffer, sizeof(SendBuffer));
      MaixSerial.flush();
      delay(100);
      while (!vision_updated)
      {
        ;
      }
      vision_updated = false; // 重置标志位
      delta_x = MaixCam.Delta_X ; // 相对位移，以车为坐标系
      delta_y = MaixCam.Delta_Y ;
      Run_PID_left(delta_y);
      run_flag=0;
    }

    isRunning = IsRunning();
    if(!isRunning && fabs(delta_y) <= MaxError_Move)
    {
      run_flag=1;
      currentState = task_2_3;
    }else if(!isRunning && fabs(delta_y) > MaxError_Move)
    {
      run_flag=1;
    }else{
      ;
    }    
    break;
  case task_2_3:
    if(run_flag)
    {
      MaixSerial.write(SendBuffer, sizeof(SendBuffer));
      MaixSerial.flush();
      delay(100);
      while (!vision_updated)
      {
        ;
      }
      vision_updated = false; // 重置标志位
      delta_x = MaixCam.Delta_X ; // 相对位移，以车为坐标系
      delta_y = MaixCam.Delta_Y ;
      Run_PID_forward(delta_x);
      run_flag=0;
    }

    isRunning = IsRunning();
    if(!isRunning && fabs(delta_x) <= MaxError_Move)
    {
      run_flag=1;
      currentState = task_2_5;
      if(fabs(delta_y) > MaxError_Move){
        currentState = task_2_4;
      }
    }else if(!isRunning && fabs(delta_x) > MaxError_Move)
    {
      run_flag=1;
    }
    else{
      ;
    }   
    break;
  case task_2_4:
   // cur_rot=Get_cur_rot();
      if(/*fabs(task[i].tar_rot-cur_rot)>MaxError_Rot&&*/run_flag){
      
      RotateCar_toTarget(cur_rot-task[i].tar_rot, 1, 1);
      run_flag=0; 
    }

    
    isRunning = IsRunning();

    if( !isRunning)       //如果有二次定位任务则进入task_2,无则进入第二阶段i++。
    {
      run_flag=1;
      currentState = task_2_2;
    }
   else{
      ;
    }
  break;

  case task_2_5:
    armStepper.runToNewPosition(100);
    delay(100);
    if(!isRunning && task[i].flag_2 == 0)       //如果有二次定位任务则进入task_3,无则进入第二阶段i++。
    {
      i++;
      run_flag=1;
      currentState = task_1_1;
    }
    else     // if(!isRunning && task[i].flag_2 == 1)
    {
      run_flag=1;
      currentState = task_3_1;
    }
    break;
  case task_3_1:
  if(i==1||i==4)//执行载物盘处手爪任务
  {
    currentState = task_3_2;
  }else if(i==2||i==5)
  {
    currentState = task_3_3;//执行粗加工处手爪运动
  }else{
    currentState = task_3_4;//执行成品区手爪运动
  }
  break;

  case task_3_2:
  for (j=(rounds-1)*3;j!=(rounds)*3;j++){
    Grab_ZhuanPan_to_Storage(qr_int_str[j]);
  }
  rotateBase(0);
  currentState=task_3_5;
  break;

  case task_3_3:
  for( dataIndex=(rounds-1)*3;dataIndex!=rounds*3;dataIndex++){
    Grab_Storage_to_Place(0);
  }
  for( dataIndex=(rounds-1)*3;dataIndex!=rounds*3;dataIndex++){
    Grab_Ground_to_Storage();
  }  
  i++;
  rotateBase(0);
  currentState=task_1_1;
  break;

  case task_3_4:
  for( dataIndex=(rounds-1)*3;dataIndex!=rounds*3;dataIndex++){
    Grab_Storage_to_Place(rounds-1);
  }
  i++;
  rotateBase(0);
  currentState=task_1_1;
  break;
  case task_3_5:
 if(/*fabs(task[i].tar_rot-cur_rot)>MaxError_Rot&&*/run_flag){
    RotateCar_toTarget(cur_rot-task[i].tar_rot, 1, 1);
      run_flag=0; 
    }
 isRunning = IsRunning();
    if(!isRunning )      
    {
      i++;
      run_flag=1;
      currentState = task_3_6;
    }
  break;

  case task_3_6:
    if(run_flag)
    {
      Run_PID_left(100);
     // delay(500);
      run_flag=0;
    }

    isRunning = IsRunning();
    if(!isRunning)
    {
      run_flag=1;
      currentState = task_1_1;
    }
  break;
  default:
    break;
}
  M1_stepper.run();
  M2_stepper.run();
  M3_stepper.run();
  M4_stepper.run();
}