// 手眼通讯协议

/*
Arduino端

发送协议
帧头0XAA + 1个字节指令+帧尾0XBB

指令格式：'C'———— 获取颜色。
指令格式：'E'———— 获取误差。
指令格式：'F'———— 待机状态。


接收协议
帧头0XAA + 5个字节数据+帧尾0XBB

5个字节数据
颜色模式(当抓取可行OPENMV才发送，OPENMV结合色块中心位置和色块大小判断)：
1字节+1字节+3字节
'C'+颜色+3个备用

误差模式(当获取到正确误差时OPENMV才发送)：
1字节+2字节+2字节
'E'+x误差（int）+y误差（int）

*/

#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>
#define TXD1 27
#define RXD1 14
#define SDA1 18
#define SCL1 5
#define WMR_I2C_ADDR 0x78  // 移动机器人从设备地址可以设置成0 ~ 127中的地址
#define OpenMV_BAUD 115200 // OpenMV通讯波特率
#define OpenMV_SERIAL Serial1
// #define QR_USE_Serial0 // 注释此行来开启串口调试输出
#ifndef QR_USE_Serial0
#define DEBUG_SERIAL
#endif

#define EYE_HEADER 0xAA // 协议帧头
#define EYE_END 0xBB    // 协议帧尾
#define EYE_DATA_SIZE 7 // 眼传回数据结构体字节数（固定值）

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
const char *ptr_qrcode = "000-000";

int error_x, error_y;
byte ActiveColor;
// 发送协议
void sendProtocol(byte instruction)
{
  OpenMV_SERIAL.write(0xAA);        // 帧头
  OpenMV_SERIAL.write(instruction); // 指令
  OpenMV_SERIAL.write(0xBB);        // 帧尾
}

// 接收协议
boolean receiveProtocol()
{
  byte buffer[EYE_DATA_SIZE];
  int bytesRead = 0;

  // 等待帧头
  while (OpenMV_SERIAL.available() > 0)
  {
    byte incomingByte = OpenMV_SERIAL.read();
    if (incomingByte == 0xAA)
    {
      // 读取剩余的数据
      buffer[bytesRead++] = incomingByte;
      while (bytesRead < EYE_DATA_SIZE && OpenMV_SERIAL.available() > 0)
      {
        buffer[bytesRead++] = OpenMV_SERIAL.read();
      }
      // 检查帧尾
      if (bytesRead == EYE_DATA_SIZE && buffer[6] == 0xBB)
      {
        // 解析模式数据和有效数据
        byte mode = buffer[1];
        if (mode == 'C')
        {
          // 处理颜色数据，第一个字节是有效数据
          ActiveColor = buffer[2];
        }
        else if (mode == 'E')
        {
          // 处理坐标数据，有效数据的前两个字节合并为x，后两个字节合并为y
          error_x = buffer[2] + buffer[3] * 256;
          error_y = buffer[4] + buffer[5] * 256;
        }
        return true; // 接收成功
      }
      else
      {
        // 帧尾错误，重置读取
        bytesRead = 0;
      }
    }
  }

  return false; // 接收失败
}

void setup()
{

  //                   baud    config      rx  tx
  OpenMV_SERIAL.begin(OpenMV_BAUD, SERIAL_8N1, RXD1, TXD1); // 二合一版
  Serial.begin(115200);
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  // https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形
}

void loop()
{
  // 发送数据
  sendProtocol('C');

  // 接收数据
  if (receiveProtocol())
  {
    Serial.println(ActiveColor);
  }

  // 发送数据
  sendProtocol('E');
  // 接收数据
  if (receiveProtocol())
  {
    Serial.print("error_x:");
    Serial.println(error_x);
    Serial.print("error_x:");
    Serial.println(error_y);
  }

  // 发送数据
  sendProtocol('F');
}
