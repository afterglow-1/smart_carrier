// 一路IIC0做主机用于屏幕显示，另一路IIC1做主机用于和移动机器人主控通讯
// http://www.taichi-maker.com/homepage/reference-index/arduino-library-index/wire-library/
// 主机给从机发送指令
/*
指令格式P+n,  P2P运动到Pn点。
指令格式X+n,X方向误差n毫米。
指令格式Y+n,Y方向误差n毫米。
指令格式Z+n,Z方向误差n分。
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
#define WMR_I2C_ADDR 0x78 // 移动机器人从设备地址可以设置成0 ~ 127中的地址
// #define QR_USE_Serial0 // 注释此行来开启串口调试输出
#ifndef QR_USE_Serial0
#define DEBUG_SERIAL
#endif

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

const char *ptr_qrcode = "000+000";

bool isSlave78Online = false;
// 定义移动机器人状态变量
char WMR_status;
byte error;
// todo 添加超时处理,目前阻塞式的，只有通讯成功才退出
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
    return _wmr_status;
  }
}

// 设置机器人P2P镇定点
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
  Serial.begin(115200);
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  // https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形

  // Wire1初始化, ESP32S做主机
  // 如果未指定地址，则以主机身份加入总线
  Wire1.begin(SDA1, SCL1, 0); 
  delay(200);                 // 等待从机上线
  // 检测移动机器人从机是否在线
  Wire1.beginTransmission(WMR_I2C_ADDR);
  error = Wire1.endTransmission();

#ifdef DEBUG_SERIAL
  Serial.println(error);
#endif
  if (error == 0)
  {
    isSlave78Online = true;
#ifdef DEBUG_SERIAL
    Serial.printf("I2C device found at address 0x%02X\n", WMR_I2C_ADDR);
#endif
  }
  else if (error != 2)
  {
#ifdef DEBUG_SERIAL
    Serial.printf("Error %d at address 0x%02X\n", error, WMR_I2C_ADDR);
#endif
  }
  // 获取移动机器人状态,如果不是准备就绪状态0x66就阻塞程序。
  if (isSlave78Online)
  {
    while (WMR_status != 0x66)
    {
      WMR_status = get_wmr_status();
    }
  }
}

void loop()
{
  if (isSlave78Online)
  {
    //do something
  }
}
