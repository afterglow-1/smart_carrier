#include <Arduino.h>

#include "FashionStar_Arm5DoF.h"
//需要添加  舵机原始角度检查
FSARM_ARM5DoF arm;  //机械臂对象

void setup() {
  Serial.begin(115200);
  arm.init();  //机械臂初始化
  arm.gripperOpen();
  delay(500);  // 等待1s
  Serial.println(arm.servos[0].queryVoltage());
  Serial.println(arm.servos[0].querySN());
  Serial.println(arm.servos[0].queryType());
  Serial.println(arm.servos[0].queryFirmware());
  Serial.println(arm.servos[1].querySN());
  Serial.println(arm.servos[2].querySN());
  Serial.println(arm.servos[3].querySN());
  Serial.println(arm.gripper_servo.querySN());
}

void loop() {}