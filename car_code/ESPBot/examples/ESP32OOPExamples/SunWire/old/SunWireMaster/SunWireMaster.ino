// ESP32 S3 做为主机
#include "Wire.h"
// Arduino -Wire库始终使用的是7位地址 最大到0x7f
// Wire库的实现使用了32字节缓冲区
#define I2C_DEV_ADDR 0x78  // 从设备地址可以设置成0 ~ 127中的地址
bool isSlave78Online = false;
byte error;
void setup() {
  Serial.begin(115200);
  // Wire初始化, 加入i2c总线
  // 如果未指定，则以主机身份加入总线。
  Wire.begin();
  delay(200);
  Wire.beginTransmission(I2C_DEV_ADDR);
  error = Wire.endTransmission();//默认是true
   Serial.println(error);
  if (error == 0) {
    isSlave78Online = true;
    Serial.printf("I2C device found at address 0x%02X\n", I2C_DEV_ADDR);
  } else if (error != 2) {
    Serial.printf("Error %d at address 0x%02X\n", error, I2C_DEV_ADDR);
  }
}

// 定义一个byte变量以便串口调试
byte x = 0;

void loop() {
  if (isSlave78Online) {
    // 将数据传送到从设备＃I2C_DEV_ADDR
    Wire.beginTransmission(I2C_DEV_ADDR);
    // 发送5个字节
    Wire.write("CMD");
    // 发送一个字节
    Wire.write(x);
    // 停止传送
    Wire.endTransmission();
    Serial.print("Send:");
    Serial.println(x);
    x++;
    delay(1000);
  }
  /*
  else {
    Wire.beginTransmission(I2C_DEV_ADDR);
    error = Wire.endTransmission();
    if (error == 0) {
      isSlave78Online = true;
      Serial.printf("I2C device found at address 0x%02X\n", I2C_DEV_ADDR);
    } else if (error != 2) {
      Serial.printf("Error %d at address 0x%02X\n", error, I2C_DEV_ADDR);
      delay(1000);
    }
  }
  */
}