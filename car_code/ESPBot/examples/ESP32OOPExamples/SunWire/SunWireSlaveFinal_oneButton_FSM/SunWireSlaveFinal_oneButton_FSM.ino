// ESP32 S3 做为从机
// 添加一键启动
/*
移动机器人状态码：
0x00  待启动
0x11 走目标点n状态，走完后跳到0x66状态
0x22 修改轮式里程计X值,修改后跳到0x11状态
0x33 修改轮式里程计Y值,修改后跳到0x11状态
0x44 修改轮式里程计Z值,修改后跳到0x11状态
0x66  机器人一键启动后准备就绪
*/
// #include <Arduino.h>
#include "OneButton.h" //https://blog.csdn.net/finedayforu/article/details/108769901
#include "Wire.h"      //https://blog.csdn.net/xq151750111/article/details/115142727
#define DEBUG_SERIAL
// Arduino -Wire库始终使用的是7位地址 最大到0x7f
// Wire库的实现使用了32字节缓冲区
#define WMR_I2C_ADDR 0x78 // 移动机器人从设备地址，可以设置成0 ~ 127中的地址
// 按键
const int START_BTN_PIN = 0;
bool start_flag = 0;
// 定义移动机器人状态变量
char WMR_status = 0x00;
char cmd; // 命令
int8_t f_n, p_n, x_n, y_n, z_n, temp_n;
void onRequest();
void onReceive();

OneButton start_btn(START_BTN_PIN, true, true); // true:按下为低电平,true上拉模式
void start_click()
{
  start_flag = 1;
  WMR_status = 0x66;
  // digitalWrite(EN_PIN, LOW);
  // save_flag=1;
}
void setup()
{
  Serial.begin(115200);
  // Wire初始化, 加入i2c总线
  // 以从机身份加入总线。
  Wire.begin(WMR_I2C_ADDR);
  Wire.onReceive(onReceive); // 收到数据后，执行onReceive
  Wire.onRequest(onRequest); // 收到需求指令，执行onRequest
  start_btn.reset();         // 清除一下按钮状态机的状态
  start_btn.attachClick(start_click);
}

void loop()
{
  switch (WMR_status)
  {
    // 待一键启动状态
  case 0x00:
    start_btn.tick();
    break;
    // 待机空闲状态
  case 0x66:
    break;
  case 0x11:
    // 走点n
    WMR_status = 0x66;
    break;
  case 0x22:
    // 修改里程计X
    WMR_status = 0x11;
    break;
  case 0x33:
    // 修改里程计Y
    WMR_status = 0x11;
    break;
  case 0x44:
    // 修改里程计Z
    WMR_status = 0x11;
    break;
  default:
    WMR_status = 0x66;
    break;
  }
  // keep watching the push button:
  start_btn.tick();
}
// 从主设备收到数据后，执行receiveEvent
void onReceive(int howMany)
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
    WMR_status = 0x11;
#ifdef DEBUG_SERIAL
    Serial.println(p_n);
#endif
    break;
  case 'X':
    x_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x22;
#ifdef DEBUG_SERIAL
    Serial.println(x_n);
#endif
    break;
  case 'Y':
    y_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x33;
#ifdef DEBUG_SERIAL
    Serial.println(y_n);
#endif
    break;
  case 'Z':
    z_n = Wire.read(); // receive byte as an integer
    WMR_status = 0x44;
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
void onRequest()
{
  Wire.write(WMR_status); // 返回机器人状态
}