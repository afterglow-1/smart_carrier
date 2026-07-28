// ESP32 S3 做为从机
/*
移动机器人状态码：
0x66  机器人一键启动后准备就绪
0x88  机器人已到位
*/
//#include <Arduino.h>
#include "Wire.h"
#define DEBUG_SERIAL
// Arduino -Wire库始终使用的是7位地址 最大到0x7f
// Wire库的实现使用了32字节缓冲区
#define WMR_I2C_ADDR 0x78 // 移动机器人从设备地址，可以设置成0 ~ 127中的地址
// 定义移动机器人状态变量
char WMR_status;
char cmd; // 命令
int8_t f_n, p_n, x_n, y_n, z_n, temp_n;
void requestEvent();
void receiveEvent();

void setup()
{
  Serial.begin(115200);
  // Wire初始化, 加入i2c总线
  // 以从机身份加入总线。
  Wire.begin(WMR_I2C_ADDR);
  Wire.onReceive(receiveEvent); // 收到数据后，执行receiveEvent
  Wire.onRequest(requestEvent); // 收到需求指令，执行requestEvent
}

void loop()
{
}
// 从主设备收到数据后，执行receiveEvent
void receiveEvent(int howMany)
{
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
    break;
  case 'P':
    p_n = Wire.read(); // receive byte as an integer
#ifdef DEBUG_SERIAL
    Serial.println(p_n);
#endif
    break;
  case 'X':
    x_n = Wire.read(); // receive byte as an integer
#ifdef DEBUG_SERIAL
    Serial.println(x_n);
#endif
    break;
  case 'Y':
    y_n = Wire.read(); // receive byte as an integer
#ifdef DEBUG_SERIAL
    Serial.println(y_n);
#endif
    break;
  case 'Z':
    z_n = Wire.read(); // receive byte as an integer
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
void requestEvent()
{
  Wire.write(WMR_status); // 返回机器人状态
}