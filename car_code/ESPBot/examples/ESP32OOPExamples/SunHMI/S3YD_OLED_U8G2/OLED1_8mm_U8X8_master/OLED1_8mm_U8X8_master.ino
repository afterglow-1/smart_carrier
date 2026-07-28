// https://blog.51cto.com/dpjcn1990/2978430#32219_u8g2print___726
//https://blog.csdn.net/qq_17351161/article/details/105177112
#include <Arduino.h>
// U8G2库源代码地址：https://github.com/olikraus/U8g2_Arduino
#include <U8x8lib.h>  //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
//https://github.com/olikraus/u8g2/wiki/u8x8reference
//适合SH1106  https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-21444122806.40.3feb3b2eJ6JJtD&id=616314176331
#include <Wire.h>
#define I2C_DEV_ADDR 0x78  // 从设备地址可以设置成0 ~ 127中的地址
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
//SH1106
//U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
//SSD1306
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
// 定义一个byte变量以便串口调试
byte x = 0;
void setup(void) {
  Serial.begin(115200);
  // Wire初始化, 加入i2c总线
  // 如果未指定，则以主机身份加入总线。
  u8x8.begin();
  u8x8.setPowerSave(0);


  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  //https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, "123+213");  //高度变体绘制双倍高度大小的字形
  
  Wire.begin();
  Serial.println("test");
}

void loop(void) {
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  //https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, "123+213");  //高度变体绘制双倍高度大小的字形
                                        //u8x8.setCursor(4, 1);
                                        //u8x8.print("321"); //汉字要用print
  // 将数据传送到从设备＃I2C_DEV_ADDR
  delay(1000);
  Wire.beginTransmission(I2C_DEV_ADDR);
  // 发送5个字节
  Wire.write("x is ");
  // 发送一个字节
  Wire.write(x);
  // 停止传送
  Wire.endTransmission();

  x++;



  delay(1000);
  Serial.println("test1");
}

/*
U8x8 字符模式
优缺点
快速
不需要内存（RAM）
不能绘制图形
不支持所有显示控制器
https://blog.csdn.net/qq_17351161/article/details/105177112
*/