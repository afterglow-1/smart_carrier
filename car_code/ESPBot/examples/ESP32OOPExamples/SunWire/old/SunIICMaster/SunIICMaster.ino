// ESP32 S3 做为主机
// 向IIC从设备询问二维码数据
/*
  程序： I2C Master
  向I2C从设备0x78询问二维码数据

*/
#include <Wire.h>
#include "SUNIIC.h"
//通讯检测PING
//todo:添加通讯超时
void IICPing(int addr)
{
  Serial.println("begin Pinging");
  char sensors_status;
  while(PING_RECV!=sensors_status)
  {
Wire.beginTransmission(addr); //开始向从设备address发出数据
Wire.write(CMD_PING); //发送ping命令字节
Wire.endTransmission(); //停止位函数
Wire.requestFrom(addr,1,1);//要求返回一个字节
sensors_status=Wire.read();
 Serial.println("PING OK");
}
}


void setup() {
  Serial.begin(115200);
  Wire.begin(); //加入i2c总线，主设备

  Wire.beginTransmission(ARM_BOARD_ADDR); //开始向从设备address发出数据
  Wire.write("UNIT"); //发送4个字节
  Wire.write(TEMP_TYPE);
  Wire.endTransmission();    //结束i2c通讯
  if (addr == 0) {
    Serial.println("begin scanning");
  }
  Wire.beginTransmission(addr);
  Wire.write(GW_GRAY_PING);
  Wire.endTransmission();

  // 查看传感器回复
  Wire.requestFrom(addr, 1);
  char ping_rep = Wire.read();
  if (ping_rep == 0x66) {
    Serial.print(addr, HEX);
    Serial.print(" ");
    Serial.println("PING OK");
  }



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
