//#define FASTLED_ESP32_SPI_BUS HSPI  //https://github.com/FastLED/FastLED/pull/1047
//#define FASTLED_ALL_PINS_HARDWARE_SPI
#include "FastLED.h"      //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED
#define NUM_LEDS 16       // LED灯珠数量
#define LED_PIN 13        // Arduino输出控制信号引脚
#define LED_TYPE WS2812B  //LED灯带型号ESP32-S3-DevKitC-1使用SK6822LED芯片 YD:WS2812B
#define COLOR_ORDER GRB   // RGB灯珠中红色、绿色、蓝色LED的排列顺序
uint8_t MaxBright = 255;
// LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS];
// 建立光带leds
void setup() {
  Serial.begin(115200);
  // 启动串行通讯
  delay(1000);
  // 稳定性等待
  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);  //初始化LED灯
  FastLED.setBrightness(MaxBright);                              //设置光带亮度
  fill_solid(leds, NUM_LEDS, CRGB::Blue);                        //将LED光带设置为同一颜色
  FastLED.show();
}

void loop() {

}