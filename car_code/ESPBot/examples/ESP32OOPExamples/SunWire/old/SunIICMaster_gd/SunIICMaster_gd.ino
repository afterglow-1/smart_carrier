/*
  程序： I2C Master
  向I2C从设备0x8询问温度数据
  并且可以设置从设备的反馈类型
  公众号：孤独的二进制 
*/
#include <Wire.h>
#define ADDRESS 0x8 //从设备地址
#define TEMP_TYPE 0 //内容 0-摄氏度 1-华氏度

void setup() {
  Serial.begin(9600);

  Wire.begin(); //加入i2c总线，主设备

  Wire.beginTransmission(ADDRESS); //开始向从设备address发出数据
  Wire.write("UNIT"); //发送4个字节
  Wire.write(TEMP_TYPE);
  Wire.endTransmission();    //结束i2c通讯
}

void loop() {
  delay(1000);

  Wire.requestFrom(ADDRESS, 1); //向从设备0x8请求，1个字节的数据

  while (Wire.available()) {
    byte temp = Wire.read();

    Serial.print("Temperature = ");
    Serial.print(temp);
    switch (TEMP_TYPE) {
      case 0:
        Serial.println(" C");
        break;
      case 1:
        Serial.println(" F");
        break;
      default:
        break;
    }
    
  }

}
