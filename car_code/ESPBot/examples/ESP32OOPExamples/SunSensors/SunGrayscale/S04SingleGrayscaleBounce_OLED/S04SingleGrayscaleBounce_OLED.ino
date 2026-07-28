/**
 * @file S03SingleGrayscaleBounce_OLED.ino 单路灰度抖动引脚外部中断测试
 * @author igcxl (igcxl@qq.com)
 * @brief  使用外部中断
 * @note
 * @version 0.5
 * @date 2022-07-16
 * @copyright Copyright © igcxl.com 2022
 * 20mm宽度 0.2m/s 100ms
 */

#include "OOPConfig.h"
#ifdef USE_IIC_OLED
#include <U8g2lib.h>  //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
    /* data=*/SDA);  // ESP32 Thing, HW I2C with pin remapping
#endif

#define SINGLEGRAY_PIN 12  //灰度传感器引脚

volatile unsigned int interruptCount = 0;  // .
volatile unsigned long fallingTime = 0;
volatile unsigned long risingTime = 0;
unsigned long previousMillis = 0;  // will store last time run
unsigned long currentMillis = 0;   // will store current time run
const long period = 5;            // period at which to run in ms
//外部中断回调函数2
void IRAM_ATTR handleISR() {
  currentMillis = millis();  // store the current time
  if (currentMillis - previousMillis >= period) {
   
      interruptCount++;
   
  }

  previousMillis = currentMillis;
}
//外部中断回调函数2
void IRAM_ATTR interruptFunctionDebounce() {
  currentMillis = millis();  // store the current time
  if (currentMillis - previousMillis >= period) {
   
      interruptCount++;
   
  }

  previousMillis = currentMillis;
}
//外部中断回调函数1
void interruptFunction() { interruptCount++; }
void interruptFunctionF() {
  fallingTime = micros();
  if (fallingTime > risingTime + 50000) {
    interruptCount++;
  }
}
void interruptFunctionR() { risingTime = micros(); }
void setup() {
  Serial.begin(115200);
  Serial.println("1Gray test");
  // OLED初始化
#ifdef USE_IIC_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf);  // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("1Gray test");
  u8g2.sendBuffer();
#endif
  pinMode(SINGLEGRAY_PIN,
          INPUT_PULLUP);  // See http://arduino.cc/en/Tutorial/DigitalPins
          //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  attachInterrupt(SINGLEGRAY_PIN, interruptFunctionDebounce, CHANGE);//http://www.taichi-maker.com/homepage/reference-index/arduino-code-reference/attachinterrupt/#:~:text=attachInterrupt%20%28%29%E5%87%BD%E6%95%B0%E6%98%AF%E7%94%A8%E4%BA%8E%E4%B8%BAArduino%E5%BC%80%E5%8F%91%E6%9D%BF%E8%AE%BE%E7%BD%AE%E5%92%8C%E6%89%A7%E8%A1%8CISR%EF%BC%88%E4%B8%AD%E6%96%AD%E6%9C%8D%E5%8A%A1%E7%A8%8B%E5%BA%8F%EF%BC%89%E7%94%A8%E7%9A%84.%20ISR%EF%BC%88%E4%B8%AD%E6%96%AD%E6%9C%8D%E5%8A%A1%E7%A8%8B%E5%BA%8F%EF%BC%89%E9%A1%BE%E5%90%8D%E6%80%9D%E4%B9%89%E5%B0%B1%E6%98%AF%E4%B8%AD%E6%96%ADArduino%E5%BD%93%E5%89%8D%E6%AD%A3%E5%9C%A8%E5%A4%84%E7%90%86%E7%9A%84%E4%BA%8B%E6%83%85%E8%80%8C%E4%BC%98%E5%85%88%E5%8E%BB%E6%89%A7%E8%A1%8C%E4%B8%AD%E6%96%AD%E6%9C%8D%E5%8A%A1%E7%A8%8B%E5%BA%8F%E3%80%82.%20%E5%BD%93%E4%B8%AD%E6%96%AD%E6%9C%8D%E5%8A%A1%E7%A8%8B%E5%BA%8F%E5%AE%8C%E6%88%90%E4%BB%A5%E5%90%8E%EF%BC%8C%E5%86%8D%E5%9B%9E%E6%9D%A5%E7%BB%A7%E7%BB%AD%E6%89%A7%E8%A1%8C%E5%88%9A%E6%89%8D%E6%89%A7%E8%A1%8C%E7%9A%84%E4%BA%8B%E6%83%85%E3%80%82.%20%E4%B8%AD%E6%96%AD%E6%9C%8D%E5%8A%A1%E7%A8%8B%E5%BA%8F%E5%AF%B9%E7%9B%91%E6%B5%8BArduino%E8%BE%93%E5%85%A5%E6%9C%89%E5%BE%88%E5%A4%A7%E7%9A%84%E7%94%A8%E5%A4%84%E3%80%82.%20%E6%88%91%E4%BB%AC%E5%8F%AF%E4%BB%A5%E4%BD%BF%E7%94%A8attachInterrupt,%28%29%E5%87%BD%E6%95%B0%EF%BC%8C%E5%88%A9%E7%94%A8Arduino%E7%9A%84%E5%BC%95%E8%84%9A%E8%A7%A6%E5%8F%91%E4%B8%AD%E6%96%AD%E7%A8%8B%E5%BA%8F%E3%80%82.%20%E4%BB%A5%E4%B8%8B%E5%88%97%E8%A1%A8%E8%AF%B4%E6%98%8E%E6%94%AF%E6%8C%81%E4%B8%AD%E6%96%AD%E7%9A%84%E5%BC%95%E8%84%9A%E6%9C%89%E5%93%AA%E4%BA%9B%EF%BC%9A.%20Arduino%E6%8E%A7%E5%88%B6%E6%9D%BF.%20%E6%94%AF%E6%8C%81%E4%B8%AD%E6%96%AD%E7%9A%84%E5%BC%95%E8%84%9A.%20Uno%2C%20Nano%2C%20Mini.
  //attachInterrupt(SINGLEGRAY_PIN, interruptFunctionF, FALLING);
  delay(20);
}

// In the loop we just display interruptCount. The value is updated by the
// interrupt routine.
void loop() {

  Serial.print("Pin was interrupted: ");
    Serial.println(interruptCount, DEC);

#ifdef USE_IIC_OLED
  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB12_tf);  // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("interruptCount:");
  u8g2.setCursor(0, 32);
  u8g2.print(interruptCount);
  u8g2.sendBuffer();
#endif


}
