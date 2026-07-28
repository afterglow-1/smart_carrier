#include "SunUPFLOW.h"
#include "FastLED.h" //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED


#define NUM_LEDS 16      // LED灯珠数量
#define LED_PIN 5       // Arduino输出控制信号引脚
#define LED_TYPE WS2812B // LED灯带型号ESP32-S3-DevKitC-1使用SK6822LED芯片 YD:WS2812B
#define COLOR_ORDER GRB  // RGB灯珠中红色、绿色、蓝色LED的排列顺序
uint8_t MaxBright = 255;
// LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS];
// ESP32
//#define TXD1 32
//#define RXD1 23
#define TXD1 27
#define RXD1 14
// #define TXD1 32
// #define RXD1 33//23
// ESP32  S3
// #define TXD2 20
// #define RXD2 19
//#define TXD1 1
//#define RXD1 45
unsigned long printTime, lastPrint;
uint16_t last_x = 0;
UPFLOW LC302GS_uart1(&Serial1, RXD1, TXD1, 460800);

void setup()
{
  Serial.begin(115200);

  Serial.println("LC302GS_uart1");
  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS); // 初始化LED灯
  FastLED.setBrightness(MaxBright);                             // 设置光带亮度
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  // fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  lastPrint = millis();
}

void loop()
{
  while (Serial1.available())
  {
    LC302GS_uart1.readData(Serial1.read()); // Call
  }
  if ((millis() - lastPrint > 200))
  // if ((last_x != LC302GS_uart1.packData.flow_x_integral))
  {
    lastPrint = millis();
    last_x = LC302GS_uart1.packData.flow_x_integral;
    Serial.println(millis());
    Serial.print("X:");
    Serial.print(LC302GS_uart1.packData.flow_x_integral);//采样间隔时间移动量
    Serial.print("X_OFFSET:");
    Serial.println(LC302GS_uart1.x_Offset);
    Serial.print("Y:");
    Serial.print(LC302GS_uart1.packData.flow_y_integral);
    Serial.print("Y_OFFSET:");
    Serial.println(LC302GS_uart1.y_Offset);
    Serial.print("T:");
    Serial.println(LC302GS_uart1.packData.integration_timespan);
  }
}
