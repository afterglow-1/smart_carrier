// 串口0 专用扫码模块扫码OLED显示测试 兼容CR 和CRLF结尾
//烧录程序时要断开和扫码模块的连接，否则会串口冲突

#include <Arduino.h>
#include <U8x8lib.h>
// 33 32 31 2B 31 32 33 0D 0A
// GM75默认是CR
// 设置后可改为CRLF
// CR（Carriage Return），回车符，用符号’\r’表示， 十进制ASCII代码是13，16进制0x0D；
// LF（Line Feed），换行符，用符号’\n’表示，十进制ASCII代码是10，16进制0x0A；
const int bufferSize = 8;      // 7字节数据 + 1字节回车符+（1字节换行符）
char receivedData[bufferSize]; // 存储接收到的数据
char str[] = "000+000";
const char *ptr_qrcode = str; // 指向常量的指针
bool scanFlag = false;
int dataIndex = 0; // 数据索引

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

void setup()
{
  Serial.begin(9600); // 设置扫码模块串口波特率为9600，串口模块默认波特率，详见模块手册
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  // https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形
}

void loop()
{
 //扫码成功才刷新
  if (scanFlag)
  {
    scanFlag = false;
    u8x8.clearDisplay();
    u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形
                                          // Serial.println("test");
  }
  
  else
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
        scanFlag = true;//数据接收成功
        // 处理接收到的数据，可以在这里添加你的处理逻辑
        ptr_qrcode = receivedData;

        // Serial.println("Received Data: " + String(receivedData)); // 打印接收到的数据
      }
    }
  }
  }
}