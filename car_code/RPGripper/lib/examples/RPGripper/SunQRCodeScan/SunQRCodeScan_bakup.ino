// 串口0 专用扫码模块扫码OLED显示测试
#include <Arduino.h>
#include <U8x8lib.h>
// 33 32 31 2B 31 32 33 0D 0A
const int bufferSize = 9;      // 7字节数据 + 1字节回车符+1字节换行符
char receivedData[bufferSize]; // 存储接收到的数据
char str[] = "000-000";
const char *ptr_qrcode = str; // 指向常量的指针
bool scanFlag=false;
int dataIndex = 0; // 数据索引

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);


void setup()
{
  Serial.begin(9600); // 设置串口波特率为9600，串口模块默认波特率，详见模块手册
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  // https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形
}

void loop()
{

  while (Serial.available())
  {
    char incomingByte = Serial.read(); // 读取一个字节数据

    if (dataIndex < (bufferSize - 1))
    {
      receivedData[dataIndex] = incomingByte; // 将数据存储到数组中
      dataIndex++;
    }

    // 检查是否接收到换行符
    if (incomingByte == 0x0A)
    {
      receivedData[dataIndex] = 0x0A; // 在数据末尾添加换号符
      dataIndex = 0; // 重置数据索引
      scanFlag=true;
      // 处理接收到的数据，可以在这里添加你的处理逻辑
      ptr_qrcode = receivedData;
      
      // Serial.println("Received Data: " + String(receivedData)); // 打印接收到的数据

    }
  }
if(scanFlag)
{ scanFlag=false;
  u8x8.clearDisplay();
  // u8x8.setFont(u8x8_font_inr21_2x4_n);  //https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, ptr_qrcode); // 高度变体绘制双倍高度大小的字形
   //Serial.println("test");
  }
}