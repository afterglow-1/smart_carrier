#include <Arduino.h>
#include <Wire.h>
// 每当接收到来自主机的数据时执行的事件函数
// 此函数被注册为事件，调用请见setup（）
void receiveEvent(int howMany)
{
    // 循环读取数据(除了最后一个字符)
    while (1 < Wire.available())
    {
        // 接收字节数据并赋值给变量c(char)
        char c = Wire.read();
        // 打印该字节
        Serial.print(c);
    }
    // 以int整数的形式接受字节数据并赋值给x(int)
    int x = Wire.read();
    // 打印该int变量x
    Serial.println(x);
} 
void setup()
{
    // Wire初始化, 并以从设备地址#78的身份加入i2c总线
    Wire.begin(0x78);
    // 注册接受事件函数
    Wire.onReceive(receiveEvent);
    // 初始化串口并设置波特率为115200
    Serial.begin(115200);
}
 
void loop()
{
    //delay(100);
}
 
