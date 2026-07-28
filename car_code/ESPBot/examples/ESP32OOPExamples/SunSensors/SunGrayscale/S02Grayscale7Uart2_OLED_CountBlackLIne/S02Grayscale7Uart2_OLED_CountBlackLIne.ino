
#include "OOPConfig.h"
//灰度传感器库,目前兼容7路/8路串口和IO口采集，输出值统一为深色为高电平（1），浅色为0。一般深色少，浅色多，统一后效率提高。
// 构造函数原型  Grayscale(byte Num = GRAYSCALE_DEFAULT_NUM, HardwareSerial
// *uartPort = &GRAYSCALE_DEFAULT_PORT,
// bool isDarkHigh = true, bool isOffset = false);  默认构造函数
// 7路灰度，仅使用串口2, Num灰度数量，仅使用指定串口 #define RX2 19 #define TX2
//请提前调节7路灰度，深色为高电平，非偏移量输出。
#ifdef USE_OLED
#include <U8g2lib.h> //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
    /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
#endif
#ifndef DEBUG_INFO
#define DEBUG_INFO
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

// 使用串口构造函数
//如果传感器浅色高电平 （黑色时低电平） 使用下方构造函数 根据调试时3号指示灯灭
// Grayscale GraySensors(/*Num*/ 7, /**uartPort*/ &Serial2, /*isDarkHigh*/
// false,
//                     /*isOffset*/ false);  // 7路灰度使用串口2
//如果传感器浅色时低电平 （黑色时高电平） 使用下方构造函数 根据调试时3号指示灯亮
// Grayscale GraySensors(/*Num*/ 7, /**uartPort*/ &Serial2, /*isDarkHigh*/ true,
//                     /*isOffset*/ false);  // 7路灰度使用串口2

Grayscale GraySensors; //也可使用默认构造函数。
// uart输出量的结构体
Grayscale::strOutput GrayUartOutputIO;
unsigned long lastRead, currentRead;
int blacklineCount = 0;
bool isGrayFirst = 1;
struct IOSTATUS
{
  unsigned long getTime; //数据获得时间
  byte getIOCount;       //数据值
};
struct IOSTATUS lastIOSTATUS, currenttIOSTATUS;

void setup()
{
  // Serial.begin(9600, SERIAL_8N1, RX, TX);
#ifdef DEBUG_INFO
  Serial.begin(BAUDRATE);
#endif
  delay(300);

#ifdef USE_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();              // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf); // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("GRAYSCALE TEST");
  u8g2.sendBuffer();
#endif

  lastRead = millis();
}

void loop()
{
  currentRead = millis();
  if (currentRead - lastRead > 5) //每5ms读取一次
  {
    lastRead = currentRead;

    GrayUartOutputIO = GraySensors.readUart(); //读取串口数字量数据
    }

  if (GrayUartOutputIO.ioCount > 5)
  {
    //先保存读数
    lastIOSTATUS.getTime = currenttIOSTATUS.getTime;
    lastIOSTATUS.getIOCount = currenttIOSTATUS.getIOCount;
    //再更新读数

    currenttIOSTATUS.getTime = millis();
    currenttIOSTATUS.getIOCount = GrayUartOutputIO.ioCount;
    if (isGrayFirst)
    {
      isGrayFirst = 0;
      blacklineCount++;
    }

    else
    {
      //间隔1秒才计数 可根据速度修改
      if (currenttIOSTATUS.getTime - lastIOSTATUS.getTime > 1000)
      {
        blacklineCount++;
      }
    }
  }

#ifdef DEBUG_INFO
  Serial.print((String) "黑色点数：" + GrayUartOutputIO.ioCount +
               "点变化数:" + GrayUartOutputIO.ioChangeCount + "偏移量：" +
               GrayUartOutputIO.offset);
  Serial.print("二进制IO值:");
  Serial.print(GrayUartOutputIO.ioDigital, BIN);
  Serial.println("Next");
  Serial.println(millis());
#ifdef USE_OLED
  u8g2.clearBuffer();                 // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB12_tf); // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("offset:");
  u8g2.print(GrayUartOutputIO.offset);
  u8g2.setCursor(0, 32);
  u8g2.print("lineCnt:");
  u8g2.print(blacklineCount);
  u8g2.setCursor(0, 48);
  u8g2.print("ioCnt:");
  u8g2.print(GrayUartOutputIO.ioCount);
  u8g2.setCursor(0, 64);
  u8g2.print("BIO:");
  u8g2.print(GrayUartOutputIO.ioDigital);
  u8g2.sendBuffer();
#endif
#endif
  // delay(2000);
}