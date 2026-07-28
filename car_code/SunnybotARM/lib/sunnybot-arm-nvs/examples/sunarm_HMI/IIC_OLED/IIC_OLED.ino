#include <Arduino.h>
#include <U8g2lib.h> //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // ESP32 Thing, HW I2C with pin remapping


void setup(void) {
  u8g2.begin();
   u8g2.enableUTF8Print();		// enable UTF8 support for the Arduino print() function 支持中文
}

void loop(void) {
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_unifont_t_chinese2);	// choose a suitable font
  u8g2.drawStr(0,10,"hello+你好");	// write something to the internal memory
  u8g2.sendBuffer();					// transfer internal memory to the display
  delay(1000);  
}
