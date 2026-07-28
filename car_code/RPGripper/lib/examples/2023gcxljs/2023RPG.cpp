/*
 * @Author: distroyer of the world 3210101752@zju.edu.cn
 * @Date: 2023-11-30 00:26:23
 * @LastEditors: igcxl acer5502@gmail.com
 * @LastEditTime: 2024-07-10 20:46:51
  */
// 一路IIC0做主机用于屏幕显示，另一路IIC1做主机用于和移动机器人主控通讯
// http://www.taichi-maker.com/homepage/reference-index/arduino-library-index/wire-library/
// 主机给从机发送指令
/*
指令格式P+n,  P2P运动到Pn点。
指令格式X+n,X方向运动n毫米。
指令格式Y+n,Y方向运动n毫米。
指令格式Z+n,Z方向运动n毫米。
指令格式F+n,获取移动机器人状态信息。
移动机器人状态码：
0x00  待启动
0x11 走目标点n状态，走完后跳到0x66状态
0x22 修改轮式里程计X值,修改后跳到0x11状态
0x33 修改轮式里程计Y值,修改后跳到0x11状态
0x44 修改轮式里程计Z值,修改后跳到0x11状态
0x66  机器人一键启动后准备就绪

*/
// todo 也可以把一键启动改到ESP32S上。用零点开关P26引脚。
#include <Arduino.h>
#include "FashionStar_UartServoProtocol.h"  // 串口总线舵机通信协议
#include "FashionStar_UartServo.h"          // Fashion Star串口总线舵机
#include <AccelStepper.h>
#include "OneButton.h"
#include <U8x8lib.h>
#include <Ticker.h> 
#include <Wire.h>
#define SDA1 18
#define SCL1 5
#define WMR_I2C_ADDR 0x78    // 移动机器人从设备地址可以设置成0 ~ 127中的地址

#define TXD1 27
#define RXD1 14

#define QQVGA_RESOLUTION_X 160
#define QQVGA_RESOLUTION_Y 108

// #define OPENMVDEBUG
// #define CALIBRATION_FEASIBILITY_CHECK

#ifndef OPENMVDEBUG
#define QRSCAN_Serial Serial // 注释此行来开启串口调试输出
#endif

#ifndef QRSCAN_Serial
#define DEBUG_SERIAL Serial
#endif

// 串口总线舵机配置
#define SERVO0 0 // 舵机0的ID号 基座舵机
#define SERVO1 1 // 舵机1的ID号 载物盘舵机
#define SERVO4 4 // 舵机4的ID号 手爪
#define SERVO_BAUDRATE 115200 // 波特率

//步进电机
#define STEPPPER_EN_PIN 32 // 使能引脚
#define dirPin 25
#define stepPin 33
#define motorInterfaceType 1 //< Stepper Driver, 2 driver pins required

#define MAX_FIND_COUNT 10

//OpenMV
#define OpenMV_SERIAL Serial1
// #define QRCODEDEBUG
enum RunState
{
  one_button_check,        // 一键启动检测
  arm_go_home,             // 机械臂回参考点
  tell_car_to_go_to_point, // QRCode_P1，storage_area_P2，roughing_area_red_P3,roughing_area_green_P4,roughing_area_blue_P5,semifinished_area_red_P7
  //semifinished_area_green_P8,semifinished_area_blue_P9,...
  scan_display_QRCode,
  tell_CAM_to_scan_position_deviation,
  send_deviatione_to_car,
  tell_CAM_to_get_object_color,
  pick_Object_by_order, // Pick and Place
  go_place_point_by_order,
  place_block,
  pick_block,
  // tell_CAM_to_scan_position_deviation,
  // send_deviatione_to_car,
  pick_place_Object_to_outside

};
enum RunState run_state = one_button_check;

enum Serial1_state
{
  serial_intergrity_check,//是否接收到'['
  x_deviation_receive,//接收x方向偏移
  receive_comma,//接收逗号
  y_deviation_receive,//接收y方向偏移
  debug_print
};

enum Serial1_color_state
{
  color_serial_intergrity_check,//是否接收到'{'
  color_message_receive,
  receive_complete

};

enum place_region
{
  rough_procession,
  precise_procession
};
enum place_region place_;

enum Serial1_state serial1_state = serial_intergrity_check; //圆心坐标

enum Serial1_color_state serial1_state_color = color_serial_intergrity_check;//颜色识别

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

bool isSlave78Online = false;
// 定义移动机器人状态变量
char WMR_status;
byte pointNUM = 0;
byte placeNUM = 0;
byte error;
const int bufferSize = 8;      // 7字节数据 + 1字节回车符+（1字节换行符）
char receivedData[bufferSize]; // 存储接收到的数据
char qrcode_str[] = "000-000";
char qrcodestr[6] = {0};
const char *ptr_qrcode = qrcode_str; // 指向常量的指针
bool scanFlag = false;
int dataIndex = 0; // 数据索引
int qr_int_str[7] = {0};

int point_num = 0;

int errcode = 0;//视觉找圆环返回的错误码
int errcode_color = 0;//颜色识别

int deviation_x = 59,deviation_y = 39;//openmv传回的偏差量
int odox,odoy;//移动机器人传回数据
int cam_scan_deviation_flag = 0;//未检测到过圆时为0,检测到过之后为1
int count = 0;

void QRcode_scanning();
int receive_deviation_from_openmv();
int get_point_number_from_qrcode_str(int n);
int receive_color_from_openmv();

bool is_transmitted = false;//是否已经传送对应点(粗加工区按顺序)

int laps = 1;//圈数

int four_times_flag = 0;
int grab_rotation_times_count=3;

int times_of_grab_from_rotation = 1;//抓取转盘上物块的次数，共六次，当前为正在抓第一个的状态
int int_current_color=0;//检测到的当前颜色值
/**步进电机*******************************************/
#define step_grab_rotation 3700 //步进电机抓取转盘物料的位置
#define step_grab_rotation_upper 1100
#define step_toward_car 1600
#define step_toward_car_upper 0
#define step_ground_place 10200
#define circle_location 8000
#define step_block_place 4600
#define step_rotation_color 3700//1600//识别颜色时的高度
/**AAAAA舵机****************************************/
// #define grip0angle 0  //抓取大角度
// #define closeangle  -90 //抓物块闭合角度
// #define openangle  40//放完返回抓取状态——避免干涉——最大角度

// #define armoutwards   -96  //机械臂底部舵机朝车外角度？
// #define arminwards    86  //机械臂底部舵机朝车上的角度//应该差180度？

// int storage[4]={0,87,-4,-94};  //1R  2G  3B  储物盘转向正对机械臂的角度

/**BBBB舵机****************************************/
#define grip0angle 0  //抓取大角度
#define closeangle  -93 //抓物块闭合角度
#define openangle  38//放完返回抓取状态——避免干涉——最大角度

#define armoutwards   -55  //机械臂底部舵机朝车外角度
#define arminwards    127  //机械臂底部舵机朝车上的角度//应该差180度

int storage[4]={0,91,-1,-89};  //1R  2G  3B  储物盘转向正对机械臂的角度
/**步进***************************************************/
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);// Create a new instance of the AccelStepper class:

/**舵机***************************************************/
// 创建舵机的通信协议对象
FSUS_Protocol protocol(SERVO_BAUDRATE);
// 创建舵机的实例
FSUS_Servo servo0(SERVO0, &protocol); // 机械臂舵机
FSUS_Servo servo1(SERVO1, &protocol); // 载物盘舵机
FSUS_Servo servo4(SERVO4, &protocol); // 手爪



// 返回值：移动机器人状态码
char get_wmr_status(void)
{
  char _wmr_status; // 移动机器人状态
  Wire1.requestFrom(WMR_I2C_ADDR, 2);
  // 当从设备接收到信息时值为true
  while (Wire1.available())
  {
    // 接收并读取从设备发来的一个字节的数据
    _wmr_status = Wire1.read();
    point_num = Wire1.read();
  }
  return _wmr_status;
}

// 设置机器人镇定点
void set_wmr_point(byte n)
{
  Wire1.beginTransmission(WMR_I2C_ADDR);
  // 发送1个字节
  Wire1.write("P"); // 发送命令“F”
  // 发送1个字节
  Wire1.write(n);
  // 停止传送
  Wire1.endTransmission(); // 产生停止位
}

// 设置机器人X补偿量,单位mm
void set_wmr_X(char n)
{
  Wire1.beginTransmission(WMR_I2C_ADDR);
  // 发送1个字节
  Wire1.write("X"); // 发送命令“F”
  // 发送1个字节
  Wire1.write(n);
  // 停止传送
  Wire1.endTransmission(); // 产生停止位
}
// 设置机器人Y补偿量,单位mm
void set_wmr_Y(char n)
{
  Wire1.beginTransmission(WMR_I2C_ADDR);
  // 发送1个字节
  Wire1.write("Y"); // 发送命令“F”
  // 发送1个字节
  Wire1.write(n);
  // 停止传送
  Wire1.endTransmission(); // 产生停止位
}

// 设置机器人Z补偿量,单位'
// 单位分
void set_wmr_Z(char n)
{
  Wire1.beginTransmission(WMR_I2C_ADDR);
  // 发送1个字节
  Wire1.write("Z"); // 发送命令“F”
  // 发送1个字节
  Wire1.write(n);
  // 停止传送
  Wire1.endTransmission(); // 产生停止位
}

void solid_calibration(char n)
{
  Wire1.beginTransmission(WMR_I2C_ADDR);
  // 发送1个字节
  Wire1.write("F"); // 发送命令“F”
  // 发送1个字节
  Wire1.write(n);
  // 停止传送
  Wire1.endTransmission(); // 产生停止位
}
#define USE_ORIGIN_SWITCH

#ifdef USE_ORIGIN_SWITCH
int ORIGIN_PIN = 26;                      // 零点开关引脚，遮挡输出高电平，灯灭
volatile unsigned int interruptCount = 0; //
bool zero_flag = 0;
#endif
void interruptFunction(){// 外部中断回调函数1
  zero_flag = 1;
  // interruptCount++;
}
void grab_rotation_single(int n_color);
void grab_rotation();
void place_ground(int n);
void grab_ground(int n);
void place_to_block(int n);
void return_qr_int();
void sendProtocol(byte instruction);

void setup()
{
#ifdef QRSCAN_Serial
  QRSCAN_Serial.begin(9600);
#endif
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);
#endif
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  // https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形

  // Wire1初始化, ESP32S做主机
  // 如果未指定地址，则以主机身份加入总线
  Wire1.begin(SDA1, SCL1, 0);
  delay(200); // 等待从机上线
  // 检测移动机器人从机是否在线
  Wire1.beginTransmission(WMR_I2C_ADDR);
  error = Wire1.endTransmission();

#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println(error);
#endif
  if (error == 0)
  {
    isSlave78Online = true;
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.printf("I2C device found at address 0x%02X\n", WMR_I2C_ADDR);
#endif
  }
  else if (error != 2)
  {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.printf("Error %d at address 0x%02X\n", error, WMR_I2C_ADDR);
#endif
  }
    /**舵机******************************************************************/
  // DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 软串口
  protocol.init();    // 舵机通信协议初始化  
  servo0.init();      // 舵机0初始化 机械臂舵机
  servo1.init();      // 舵机1初始化 储物盘
  servo4.init();      // 舵机4初始化 手爪

  /**步进********************************************************************/
  pinMode(STEPPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPPER_EN_PIN, LOW);      // 使能步进电机 低电平有效
  stepper.setMaxSpeed(60000);//还可以更快吗？文档说不建议超过1000
  stepper.setAcceleration(60000);

  servo0.setSpeed(500);      // 舵机0初始化 机械臂舵机
  servo1.setSpeed(500);      // 舵机1初始化 储物盘
  servo4.setSpeed(600);  
  // ticker1.attach(1, callback1); // 每1秒调用callback1
  // ticker4.attach(4, callback4); // 每4秒调用callback1
  pinMode(ORIGIN_PIN,INPUT); // See http://arduino.cc/en/Tutorial/DigitalPins
  attachInterrupt(ORIGIN_PIN, interruptFunction, RISING);
}

char show_run_state[10] = {0};
char odo_string[10] = {0};

void loop()
{
#ifdef OPENMVDEBUG
  receive_deviation_from_openmv();
#endif

#ifndef OPENMVDEBUG
  QRcode_scanning();
  if (isSlave78Online)
  {
    switch (run_state)
    {
    case one_button_check:

#ifndef CALIBRATION_FEASIBILITY_CHECK
      if (get_wmr_status() == 0x66)
      {
        run_state = arm_go_home;
      }
#endif
#ifdef  CALIBRATION_FEASIBILITY_CHECK
      if (wmr_status_ == 0x66)
      {
        run_state = tell_CAM_to_scan_position_deviation;
        set_wmr_point(0);
      }
#endif

      break;

    case arm_go_home:

    
      // armgohome
      if (true)
      {
        pointNUM = 2;
        run_state = tell_car_to_go_to_point;
        set_wmr_point(pointNUM);
      }

      break;

    case tell_car_to_go_to_point:
      if (get_wmr_status() == 0x66)//完成走点之后再前往下一点
      {
        switch (pointNUM)
        {
        case 0:
          pointNUM = 1;
          do
          {
            get_wmr_status();//获取移动机器人状态信息。
            set_wmr_point(pointNUM);//设置机器人镇定点
            //get_wmr_status();//获取移动机器人状态信息。放在此处是否更好？
          }while(point_num!=1);//最少设置两次
          break;
        case 1:
          run_state = scan_display_QRCode;
          break;
        case 2:
          pointNUM = 3;
          stepper.runToNewPosition(step_rotation_color);
          servo4.setAngle(grip0angle);
          servo0.setAngle(armoutwards);
          run_state = tell_car_to_go_to_point;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(get_wmr_status()!=0x11);
          
        case 3:
          if(get_wmr_status() == 0x66)
          {
            run_state = tell_CAM_to_get_object_color;
            while(Serial1.available())
            {
              Serial1.read();
            }
          }
          break;
        case 4:
          pointNUM = 5;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=5);
          break;
        case 5:
          pointNUM = 6;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=6);
          //机械臂到圆环识别位置
          servo0.setAngle(armoutwards);
          servo0.wait();
          stepper.runToNewPosition(circle_location);
          break;
        case 6:
          place_ = rough_procession;
          run_state = tell_CAM_to_scan_position_deviation;
          while(Serial1.available())
          {
            Serial1.read();
          }
          break;
        case 10:
          pointNUM = 11;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=11);
        break;
        case 11:
          pointNUM = 12;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=12);
          //机械臂到圆环识别位置
          servo0.setAngle(armoutwards);
          servo0.wait();
          servo4.setAngle(grip0angle);
          servo4.wait();
          stepper.runToNewPosition(circle_location);    
        break;

        case 12:
          place_ = precise_procession;
          run_state = tell_CAM_to_scan_position_deviation;
          while(Serial1.available())
          {
            Serial1.read();
          }
          break;

        case 16:
        pointNUM = 17;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=17);
        break;

        case 17:
        pointNUM = 18;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=18);
        break;

        case 18:
        laps = 2;
        place_ = rough_procession;
        pointNUM = 19;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=19);
        break;

        case 19:
        if(get_wmr_status() == 0x66)
        {
          run_state = tell_CAM_to_scan_position_deviation;
          while(Serial1.available())
          {
            Serial1.read();
          }
        }
        break;

        case 20:
        
        pointNUM = 21;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=21);
          stepper.runToNewPosition(step_grab_rotation_upper);
        break;

        case 21:
        pointNUM = 22;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=22);
        break;

        case 22:
        if(get_wmr_status() == 0x66)
        {
          run_state = tell_CAM_to_get_object_color;
          while(Serial1.available())
          {
            Serial1.read();
          }
        }
        break;

        case 23:
        pointNUM = 24;
        do
        {
          get_wmr_status();
          set_wmr_point(pointNUM);
        }while(point_num!=24);
        break;
        }
      }

      break;

    case scan_display_QRCode:
    if(scanFlag == true)
    {
      pointNUM = 2;
      run_state = tell_car_to_go_to_point;
      set_wmr_point(pointNUM);
      return_qr_int();
    }
    else
    {
      run_state = scan_display_QRCode;
    }
    break;
    

    case tell_CAM_to_get_object_color://每次到转盘前抓三个，只执行一次case

      sendProtocol('C');
      //抓取转盘（grab_rotation_times_count对第一圈和第二圈做了区分）
      while(grab_rotation_times_count!=0){
        errcode_color = receive_color_from_openmv();
        if(errcode_color == 0){//errcode_color表示是否正确读取openmv的数据
          grab_rotation();
        }
      }
      
      
      
      if (true)
      {
        pointNUM = 4;
        run_state = tell_car_to_go_to_point;
        grab_rotation_times_count = 3;
        do
        {
          get_wmr_status();
          set_wmr_point(pointNUM);
        }while(point_num!=4);
      }

      break;

    case tell_CAM_to_scan_position_deviation:
    sendProtocol('E');
    errcode = receive_deviation_from_openmv();
    if(get_wmr_status() == 0x66&&errcode == 0)
    {
      set_wmr_X((int)(-0.25*deviation_x));
      set_wmr_Y((int)(0.25*deviation_y));
      cam_scan_deviation_flag = 1;
    }
    else if(((cam_scan_deviation_flag == 0&&millis()%100 == 0))||count < MAX_FIND_COUNT)
    {
      set_wmr_Y(-1);
      count++;
    }
    if (abs(deviation_x) < 3 && abs(deviation_y) < 3)
    {
      cam_scan_deviation_flag = 0;
      count = 0;
      while(get_wmr_status()!=0x66);
      if(pointNUM == 6||pointNUM == 12)run_state = go_place_point_by_order;
      else if(pointNUM == 19)
      {
        run_state = tell_car_to_go_to_point;
        pointNUM = 20;
        do
        {
          get_wmr_status();
          set_wmr_point(pointNUM);
        }while(point_num!=20);
      }
      if(place_== rough_procession)solid_calibration(0);
      else if(place_== precise_procession)solid_calibration(1);
      deviation_x = 59;
      deviation_y = 39;
    }
    break;

    case go_place_point_by_order:
    
    if(placeNUM!=3)
    {
      if(is_transmitted == false)
      {
        pointNUM = get_point_number_from_qrcode_str(placeNUM);
        do
        {
          set_wmr_point(pointNUM);
          get_wmr_status();
        }while(point_num!=pointNUM);
        is_transmitted = true;
      }
      //三点放置过程中手臂提前准备
      stepper.runToNewPosition(step_toward_car);
      servo4.setAngle(openangle);
      servo0.setAngle(arminwards);
      //控制载物盘的角度
      if(place_== rough_procession){
        servo1.setAngle(storage[pointNUM-6]);
      }else if(place_== precise_procession){
        servo1.setAngle(storage[pointNUM-12]);
      }
      
      if(get_wmr_status() == 0x66)
      {
        servo0.setAngle(arminwards);//优化手抓时间
        run_state = place_block;
      }
    }
    else
    {
        if(place_ == rough_procession)run_state = pick_Object_by_order;
        else if(place_ == precise_procession&&laps == 1)
        {
          run_state = tell_car_to_go_to_point;
          pointNUM = 16;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=pointNUM);
        }
        else if(place_ == precise_procession&&laps == 2)
        {
          run_state = tell_car_to_go_to_point;
          pointNUM = 23;
          do
          {
            get_wmr_status();
            set_wmr_point(pointNUM);
          }while(point_num!=pointNUM);
        }
        placeNUM = 0;
    } 
    break;  

    case place_block://放置物块
    //以下对码垛做了区分
    if(laps==1){
      place_ground(placeNUM+1+(laps-1)*3);
    }else if(laps==2 && place_==rough_procession){
      place_ground(placeNUM+1+(laps-1)*3);
    }else if(laps==2 && place_==precise_procession){
      place_to_block(placeNUM+1+(laps-1)*3);
    }else{ 
    }
    if(true)
    {
      run_state = go_place_point_by_order;
      is_transmitted = false;
      placeNUM++;
      if(placeNUM == 3)
      {
        servo1.setAngle(0);
      }
    }
    break;

    case pick_Object_by_order://三点抓取物块
    if(placeNUM!=3)
    {
      if(is_transmitted == false)
      {
        pointNUM = get_point_number_from_qrcode_str(placeNUM);
        do
        {
          set_wmr_point(pointNUM);
          get_wmr_status();
        }while(point_num!=pointNUM);
        is_transmitted = true;
      }
      servo0.setAngle(armoutwards);
      servo0.wait();
      stepper.runToNewPosition(step_ground_place);
      if(get_wmr_status() == 0x66)
      {
        run_state = pick_block;
      }
    }
    else
    {
        run_state = tell_car_to_go_to_point;
        placeNUM = 0;
        pointNUM = 10;
        do
        {
          get_wmr_status();
          set_wmr_point(pointNUM);
        }while(point_num!=pointNUM);
    } 
    break;

    case pick_block:
    grab_ground(placeNUM+1+(laps-1)*3);
    if(true)
    {
      run_state = pick_Object_by_order;
      is_transmitted = false;
      placeNUM++;
    }
    break;
    }  
  }
}
#endif


int get_point_number_from_qrcode_str(int n)
{
  int pointnum;
  if(laps == 1)//第一圈
  {
    if(place_ == rough_procession)
    {
      switch(receivedData[n])
      {
        case '1':
        pointnum = 7;
        break;

        case '2':
        pointnum = 8;
        break;

        case '3':
        pointnum = 9;
        break;
      }
    }
    else if(place_ == precise_procession)
    {
      switch(receivedData[n])
      {
        case '1':
        pointnum = 13;
        break;

        case '2':
        pointnum = 14;
        break;

        case '3':
        pointnum = 15;
        break;
      }    
    }
  }
  else if(laps == 2)
  {
    if(place_ == rough_procession)
    {
      switch(receivedData[n+4])
      {
        case '1':
        pointnum = 7;
        break;

        case '2':
        pointnum = 8;
        break;

        case '3':
        pointnum = 9;
        break;
      }
    }
    else if(place_ == precise_procession)
    {
      switch(receivedData[n+4])
      {
        case '1':
        pointnum = 13;
        break;

        case '2':
        pointnum = 14;
        break;

        case '3':
        pointnum = 15;
        break;
      }    
    }
  }

  return pointnum;
}

int receive_deviation_from_openmv()//fyj的类似JSON通信协议
{
  static int _raw_x = 0,_raw_y = 0;
  if(!Serial1.available())
  {
    return 1;
  }
  while(Serial1.available())
  {
    byte x = Serial1.read();
    switch(serial1_state)
    {
      case serial_intergrity_check:
      _raw_x = 0;
      _raw_y = 0;
      if(x == '[')
      {
        serial1_state = x_deviation_receive;
      }
      break;

      case x_deviation_receive:
      if(x != ',')
      {
        _raw_x = _raw_x*10 + x - '0';
      }
      else 
      {
        serial1_state = y_deviation_receive;
      }
      break;

      case y_deviation_receive:
      if(x != ']')
      {
        _raw_y = _raw_y*10 + x - '0';
      }
      else 
      {
        serial1_state = debug_print;
      }
      break;

      case debug_print:
      deviation_x = _raw_x - QQVGA_RESOLUTION_X/2;
      deviation_y = _raw_y - QQVGA_RESOLUTION_Y/2;
      serial1_state = serial_intergrity_check;
      return 0;
      break;
    }
  }
  return 1;
}

int receive_color_from_openmv()//仿照上面的通信协议接收
{
  if(!Serial1.available())
  {
    return 1;
  }
  while(Serial1.available())
  {
    byte byte_current_color = Serial1.read();
    switch(serial1_state_color)
    {
      case color_serial_intergrity_check:
      if(byte_current_color == '{')
      {
        serial1_state_color = color_message_receive;
      }
      break;

      case color_message_receive:
      if(byte_current_color == '0'||byte_current_color == '1'||byte_current_color == '2'||byte_current_color == '3')
      {
        int_current_color = byte_current_color -'0';
        serial1_state_color = receive_complete;
      }
      else 
      {
        serial1_state_color = color_serial_intergrity_check;
      }
      break;

      case receive_complete:
      if(byte_current_color == '}')
      {
        serial1_state_color = color_serial_intergrity_check;
        return 0;
      }
      break;

      default:
      serial1_state_color = color_serial_intergrity_check;
      break;
    }
    
    return 1;
  }
  return 1;
}

void sendProtocol(byte instruction)//向openmv发送
{
  OpenMV_SERIAL.write(0xAA);        // 帧头
  OpenMV_SERIAL.write(instruction); // 指令
  OpenMV_SERIAL.write(0xBB);        // 帧尾
}

void QRcode_scanning()//二维码扫描
{
  if(scanFlag == false)
  {
    
    while (Serial.available())
    {
      char incomingByte = Serial.read(); // 读取一个字节数据
      // 检查是否接收到换行符，如果是换行符则重新开始
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
        if (incomingByte == 0x0D)
        {
          receivedData[dataIndex] = '\0'; // 在数据末尾添加字符串结束符
          dataIndex = 0;                  // 重置数据索引
          scanFlag = true;                // 数据接收成功
          // 处理接收到的数据
          ptr_qrcode = receivedData;
          u8x8.clearDisplay();
          u8x8.setFont(u8x8_font_inr21_2x4_n);  //https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
          u8x8.draw1x2String(0, 0, receivedData);
          return_qr_int();
        }
      }
    }
  } 
  else return;
}
void return_qr_int(){//syf
  int i=0;
  for(i=0;i<3;i++){
    switch(receivedData[i]){//qr_int_str 123
      case '1':qr_int_str[i+1]=1;break;
      case '2':qr_int_str[i+1]=2;break;
      case '3':qr_int_str[i+1]=3;break;
    }
  }
  for(i=4;i<7;i++){
    switch(receivedData[i]){//qr_int_str 456
      case '1':qr_int_str[i]=1;break;
      case '2':qr_int_str[i]=2;break;
      case '3':qr_int_str[i]=3;break;
    }
  }
}
void grab_rotation_single(int n_color){//单次抓取动作组
  //形参是123=>RGB
  stepper.runToNewPosition(step_grab_rotation);
  servo4.setAngle(closeangle);
  servo4.wait();
  stepper.runToNewPosition(step_grab_rotation_upper);
  servo0.setAngle(arminwards);
  servo1.setAngle(storage[n_color]);
  servo0.wait();
  servo1.wait();
  stepper.runToNewPosition(step_toward_car);
  servo4.setAngle(openangle);
  servo4.wait();
  stepper.runToNewPosition(step_rotation_color);
}

char debug_[10] = {0};

void grab_rotation(){//从地上抓到车上  n为1~6
  
  servo4.setAngle(openangle);
  servo0.setAngle(armoutwards);
  servo0.wait();
  servo4.wait();//颜色识别的姿势
  if(int_current_color == qr_int_str[times_of_grab_from_rotation]){
    grab_rotation_single(int_current_color);
    times_of_grab_from_rotation++;
    grab_rotation_times_count--;
  }
  
}
void place_ground(int n){
  int j=0;//储存1 or 2 or 3 => R or G or B
  j=qr_int_str[n];
  
  stepper.runToNewPosition(step_toward_car);
  servo4.setAngle(openangle);
  servo4.wait();
  servo1.setAngle(storage[j]);
  servo0.setAngle(arminwards);
  servo0.wait();
  servo1.wait();
  stepper.runToNewPosition(step_toward_car);
  servo4.setAngle(closeangle);
  servo4.wait();
  stepper.runToNewPosition(step_toward_car_upper);
  servo0.setAngle(armoutwards);
  servo0.wait();
  stepper.runToNewPosition(step_ground_place);
  servo4.setAngle(grip0angle);
  servo4.wait();
}
void grab_ground(int n){//从地上抓到车上
  int j=0;//储存1 or 2 or 3 => R or G or B
  j=qr_int_str[n];
  servo1.setAngle(storage[j]);
  servo0.setAngle(armoutwards);
  servo1.wait();
  servo0.wait();
  stepper.runToNewPosition(step_ground_place);
  servo4.setAngle(closeangle);
  servo4.wait();
  stepper.runToNewPosition(step_toward_car_upper);
  servo0.setAngle(arminwards);
  servo0.wait();
  stepper.runToNewPosition(step_toward_car);
  servo4.setAngle(grip0angle);
  servo4.wait();
  stepper.runToNewPosition(step_toward_car_upper);

}
void place_to_block(int n){
  int j=0;//储存1 or 2 or 3 => R or G or B
  j=qr_int_str[n];
  stepper.runToNewPosition(step_toward_car);
  servo4.setAngle(openangle);
  servo4.wait();
  servo1.setAngle(storage[j]);
  servo0.setAngle(arminwards);
  servo0.wait();
  servo1.wait();
  stepper.runToNewPosition(step_toward_car);
  servo4.setAngle(closeangle);
  servo4.wait();
  stepper.runToNewPosition(step_toward_car_upper);
  servo0.setAngle(armoutwards);
  servo0.wait();
  stepper.runToNewPosition(step_block_place);
  servo4.setAngle(grip0angle);
  servo4.wait();

}
