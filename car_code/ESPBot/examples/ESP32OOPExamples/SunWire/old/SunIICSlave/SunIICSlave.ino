/*
  程序： I2C Slave
  I2C从设备地址0x8
  公众号：孤独的二进制 
*/
#include <Wire.h>
#define ADDRESS 0x8
void requestEvent();
void receiveEvent();

byte temp = 24; //摄氏度
byte unitType = 0;  //0-摄氏度 1-华氏度

void setup() {
  Wire.begin(ADDRESS); //从设备，地址十六进制的8 
  Wire.onReceive(receiveEvent); //收到数据后，执行receiveEvent
  Wire.onRequest(requestEvent);  //收到需求指令，执行requestEvent
}

void loop() {
}

//从主设备收到数据后，执行receiveEvent
void receiveEvent(int howMany)
{
  String cmd = ""; //UNIT0-5 bytes
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read();
    cmd.concat(c);
  }
  Serial.println(cmd);
  if (cmd == "UNIT") unitType = Wire.read();    // receive byte as an integer
  cmd = "";
}

//当收到需求指令的时候，执行requestEvent函数内容
void requestEvent() {
  switch (unitType) {
  case 0:
    Wire.write(temp); //返回摄氏度
    break;
  case 1:
    Wire.write(byte((temp*1.8)+32));  //返回华氏度
    break;
  default:
    break;
}
 }