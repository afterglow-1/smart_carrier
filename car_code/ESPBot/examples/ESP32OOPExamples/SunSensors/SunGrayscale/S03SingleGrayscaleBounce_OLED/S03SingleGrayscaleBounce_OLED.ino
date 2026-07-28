/**
 * @file S03SingleGrayscaleBounce_OLED.ino 单路灰度抖动引脚更改中断测试
 * @author igcxl (igcxl@qq.com)
 * @brief
 * 通过按键控制串口0输出
 * @note
 * 按键输入引脚-需要是支持引脚更改中断引脚
 * 1.测试按键的抖动，使用EnableInterrupt库，引脚更改中断
 * See https://github.com/GreyGnome/EnableInterrupt and the README.md for more
 * information.
 * @version 0.5
 * @date 2021-03-18
 * @copyright Copyright © igcxl.com 2021
 *
 */

#include "OOPConfig.h"
#ifdef USE_OLED
#include <U8g2lib.h>  //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
    /* data=*/SDA);  // ESP32 Thing, HW I2C with pin remapping
#endif

#define SINGLEGRAY_PIN 12  //灰度传感器引脚

volatile unsigned int interruptCount = 0;  // .

unsigned long previousMillis = 0;  // will store last time run
unsigned long currentMillis = 0;   // will store current time run
const long period = 10;            // period at which to run in ms
//外部中断回调函数2
void interruptFunctionDebounce() {
  currentMillis = millis();  // store the current time
  if (currentMillis - previousMillis >= period) {
    if ( digitalRead(SINGLEGRAY_PIN))
    {
         interruptCount++;
    }    
    
  }

  previousMillis = currentMillis;
}
//外部中断回调函数1
void interruptFunction() { interruptCount++; }

void setup() {
  Serial.begin(115200);
  Serial.println("1Gray test");
  // OLED初始化
#ifdef USE_OLED
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
  attachInterrupt(SINGLEGRAY_PIN, interruptFunction, RISING);
  delay(20);
}

// In the loop we just display interruptCount. The value is updated by the
// interrupt routine.
void loop() {
  Serial.println("---------------------------------------");
  Serial.print("Pin was interrupted: ");
  Serial.print(interruptCount, DEC);
  Serial.println(" times so far.");
#ifdef USE_OLED
  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB12_tf);  // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("interruptCount:");
  u8g2.setCursor(0, 32);
  u8g2.print(interruptCount);
  u8g2.sendBuffer();
#endif

  delay(1000);
}
