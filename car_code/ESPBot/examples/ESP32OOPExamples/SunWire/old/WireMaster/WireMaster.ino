//ESP32 S3 做为主机
#include "Wire.h"
//Arduino -Wire库始终使用的是7位地址 最大到0x7f
//Wire库的实现使用了32字节缓冲区
#define I2C_DEV_ADDR 0x78 //从设备地址

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
   // Wire初始化, 加入i2c总线
    // 如果未指定，则以主机身份加入总线。
  Wire.begin();
}

void loop() {
  delay(5000);

  //Write message to the slave
  // 向从设备发送
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.printf("Hello World! %u", i++);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);
  
  //Read 16 bytes from the slave
   // 向从设备＃请求16个字节

  uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
  Serial.printf("requestFrom: %u\n", bytesReceived);
  if((bool)bytesReceived){ //If received more than zero bytes
    uint8_t temp[bytesReceived];
    Wire.readBytes(temp, bytesReceived);
    log_print_buf(temp, bytesReceived);
  }
}
