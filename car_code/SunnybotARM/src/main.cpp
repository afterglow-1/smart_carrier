#include <Arduino.h>
#include <U8x8lib.h>  
#include <Wire.h>
#define SDA1 18
#define SCL1 5
#define I2C_DEV_ADDR 0x78  // 从设备地址可以设置成0 ~ 127中的地址
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

const char *ptr_qrcode="000-000";
// 每当接收到来自主机的数据时执行的事件函数
// 此函数被注册为事件，调用请见setup（）
void receiveEvent(int howMany)
{
    // 循环读取数据(除了最后一个字符)
    while (1 < Wire1.available())
    {
        // 接收字节数据并赋值给变量c(char)
        char c = Wire1.read();
        // 打印该字节
        Serial.print(c);
    }
    // 以int整数的形式接受字节数据并赋值给x(int)
    int x = Wire1.read();
    // 打印该int变量x
    Serial.println(x);
} 
void setup()
{
    Serial.begin(115200);
  // Wire初始化, 加入i2c总线
  // 如果未指定，则以主机身份加入总线。
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_inr21_2x4_n);  //https://github.com/olikraus/u8g2/wiki/fntlist8x8 宽度2*8=16*7=112高度4*8=32
  u8x8.draw1x2String(0, 0, ptr_qrcode);  //高度变体绘制双倍高度大小的字形
  
    // Wire1初始化, 并以从设备地址#78的身份加入i2c总线
    Wire1.begin(0x78,SDA1,SCL1,0);
    // 注册接受事件函数
    Wire1.onReceive(receiveEvent);
    // 初始化串口并设置波特率为115200
    //Serial.begin(115200);
    Serial.println("test");
}
 
void loop()
{
    //delay(100);
}
 
