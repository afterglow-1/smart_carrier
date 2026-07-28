
#include "SunUPFLOW.h"
#define USE_OLED
#ifdef USE_OLED
#include <U8g2lib.h> //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
    /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
#endif

#include "FastLED.h" //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED
// ESP32
#define TXD2 17
#define RXD2 16
#define TXD1 32
#define RXD1 23
#define NUM_LEDS 16      // LED灯珠数量
#define LED_PIN 19      // Arduino输出控制信号引脚
#define LED_TYPE WS2812B // LED灯带型号ESP32-S3-DevKitC-1使用SK6822LED芯片 YD:WS2812B
#define COLOR_ORDER GRB  // RGB灯珠中红色、绿色、蓝色LED的排列顺序
uint8_t MaxBright = 255;
// LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS];
// 建立光带leds
// #define TXD1 32
// #define RXD1 33//23
// ESP32  S3
// #define TXD2 20
// #define RXD2 19
// #define TXD1 1
// #define RXD1 45
unsigned long printTime, lastPrint;
uint16_t last_x = 0;
UPFLOW LC302GS_uart2(&Serial2, RXD2, TXD2, 345600);
UPFLOW LC302_uart1(&Serial1, RXD1, TXD1, 19200);
void setup()
{
  Serial.begin(115200);

  Serial.println("LC302GS_uart2");

#ifdef USE_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();              // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf); // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("TEST");
  u8g2.sendBuffer();
#endif
  delay(100);
  // 稳定性等待
  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS); // 初始化LED灯
  FastLED.setBrightness(MaxBright);                             // 设置光带亮度
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  // fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  // 基准时间
  lastPrint = millis();
}

void loop()
{
      while (Serial1.available())
  {
    LC302_uart1.readData(Serial1.read()); // Call
  }
  while (Serial2.available())
  {
    LC302GS_uart2.readData(Serial2.read()); // Call
  }

  if ((millis() - lastPrint > 1000))
  // if ((last_x != LC302GS_uart2.packData.flow_x_integral))
  {
    lastPrint = millis();
    last_x = LC302GS_uart2.packData.flow_x_integral;
    Serial.println(millis());
    Serial.print("2X:");
    // Serial.print(LC302GS_uart2.packData.flow_x_integral);
    Serial.print(LC302GS_uart2.flow_x_integral_filtered);
    Serial.print("2X_OFFSET:");
    Serial.println(LC302GS_uart2.x_Offset);
    Serial.print("2Y:");
    // Serial.print(LC302GS_uart2.packData.flow_y_integral);
    Serial.print(LC302GS_uart2.flow_y_integral_filtered);
    Serial.print("2Y_OFFSET:");
    Serial.println(LC302GS_uart2.y_Offset);
        Serial.print("1X:");
    // Serial.print(LC302GS_uart2.packData.flow_x_integral);
    Serial.print(LC302_uart1.flow_x_integral_filtered);
    Serial.print("1X_OFFSET:");
    Serial.println(LC302_uart1.x_Offset);
    Serial.print("1Y:");
    // Serial.print(LC302GS_uart2.packData.flow_y_integral);
    Serial.print(LC302_uart1.flow_y_integral_filtered);
    Serial.print("1Y_OFFSET:");
    Serial.println(LC302_uart1.y_Offset);
// Serial.print("T:");
// Serial.println(LC302GS_uart2.packData.integration_timespan);
#ifdef USE_OLED
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB12_tf); // choose a suitable font
    u8g2.setCursor(0, 16);
    u8g2.print("2X:");
    u8g2.print(LC302GS_uart2.x_Offset);
    u8g2.setCursor(0, 32);
    u8g2.print("2Y:");
    u8g2.print(LC302GS_uart2.y_Offset);
    u8g2.setCursor(0, 48);
    u8g2.print("1X:");
    u8g2.print(LC302_uart1.x_Offset);
    u8g2.setCursor(0, 64);
    u8g2.print("1Y:");
    u8g2.print(LC302_uart1.y_Offset);
    u8g2.sendBuffer();
#endif
  }
}
