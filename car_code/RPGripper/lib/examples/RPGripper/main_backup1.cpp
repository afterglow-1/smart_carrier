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
#include <U8x8lib.h>
#include <Wire.h>
#define SDA1 18
#define SCL1 5
#define WMR_I2C_ADDR 0x78    // 移动机器人从设备地址可以设置成0 ~ 127中的地址
#define QRSCAN_Serial Serial // 注释此行来开启串口调试输出
#ifndef QRSCAN_Serial
#define DEBUG_SERIAL Serial
#endif
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
  pick_place_Object_from_storage_area, // Pick and Place
  // tell_CAM_to_scan_position_deviation,
  // send_deviatione_to_car,
  pick_place_Object_to_outside

};
enum RunState run_state = one_button_check;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

bool isSlave78Online = false;
// 定义移动机器人状态变量
char WMR_status;
byte pointNUM = 0;
byte error;
const int bufferSize = 8;      // 7字节数据 + 1字节回车符+（1字节换行符）
char receivedData[bufferSize]; // 存储接收到的数据
char str[] = "000-000";
const char *ptr_qrcode = str; // 指向常量的指针
bool scanFlag = false;
int dataIndex = 0; // 数据索引

// 返回值：移动机器人状态码
char get_wmr_status(void)
{
  char _wmr_status; // 移动机器人状态
                    // 向从设备发送， 将数据传送到从设备＃WMR_I2C_ADDR
  Wire1.beginTransmission(WMR_I2C_ADDR);
  // 发送1个字节
  Wire1.write("F"); // 发送命令“F”
                    // 发送1个字节
  Wire1.write(0);
  //  停止传送
  Wire1.endTransmission(0); // 不产生停止位
  // 向从设备＃WMR_I2C_ADDR请求1个字节
  Wire1.requestFrom(WMR_I2C_ADDR, 1);
  // 当从设备接收到信息时值为true
  while (Wire1.available())
  {
    // 接收并读取从设备发来的一个字节的数据
    _wmr_status = Wire1.read();
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

void setup()
{
#ifdef QRSCAN_Serial
  QRSCAN_Serial.begin(9600);
#endif
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
}

void loop()
{
  if (isSlave78Online)
  {
    switch (run_state)
    {
    case one_button_check:

      if (get_wmr_status() == 0x66)
      {
        run_state = arm_go_home;
      }

      break;

    case arm_go_home:

    
      // armgohome
      if (true)
      {
        pointNUM = 1;
        run_state = tell_car_to_go_to_point;
      }

      break;

    case tell_car_to_go_to_point:
      set_wmr_point(pointNUM);
      if (get_wmr_status() == 0x66)
      {
        switch (pointNUM)
        {
        case 0:
          // run_state = scan_display_QRCode;
          break;
        case 1:
          run_state = scan_display_QRCode;
          break;
        case 2:
          run_state = tell_CAM_to_get_object_color;
          break;
        }
      }

      break;

    case scan_display_QRCode:
      // scan_display_QRCode

      if (scanFlag)

      {
        // 显示
        u8x8.clearDisplay();
        // u8x8.setFont(u8x8_font_inr21_2x4_n);  //https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
        u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形

        pointNUM = 2;
        run_state = tell_car_to_go_to_point;
      }

      break;

    case tell_CAM_to_get_object_color:

      // tell_CAM_to_get_object_color
      if (true)
      {
        run_state = scan_display_QRCode;
      }

      break;

   
    }
  }
}

// 伪串口0中断 更新扫码数据
void serialEvent()
{
  // 只处理一次成功的扫码
  if (scanFlag == false)
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
        }
      }
    }

    if (scanFlag)

      {   // 显示
        u8x8.clearDisplay();
        // u8x8.setFont(u8x8_font_inr21_2x4_n);  //https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
        u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形
      }
  }
 
}
