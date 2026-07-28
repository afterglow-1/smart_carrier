#include <Arduino.h>

#include "FashionStar_Arm5DoF.h"
//需要添加  舵机原始角度检查 使用原始角度运行 需要断电重启
FSARM_ARM5DoF arm;  //机械臂对象

void setup() {
  Serial.begin(115200);
  arm.init();  //机械臂初始化
  arm.gripperOpen(0);
  delay(500);  // 等待1s
  Serial.println(arm.servos[0].queryVoltage());
  Serial.println(arm.servos[0].queryStatus(), BIN);
  Serial.println(arm.servos[1].queryStatus(), BIN);
  Serial.println(arm.servos[2].queryStatus(), BIN);
  Serial.println(arm.servos[3].queryStatus(), BIN);
  Serial.println(arm.servos[4].queryStatus(), BIN);
  /**
   * 舵机状态 数据类型：uint8_t。SUNNY ADD
   * BIT[0]-执行指令中置 1，执行完毕后清零。
   * BIT[1]-执行指令错误置 1，下次正确执行后清零。
   * BIT[2]-堵转错误置 1，堵转解除后清零。
   * BIT[3]-电压高压置 1，电压正常后清零。
   * BIT[4]-电压低压置 1，电压正常后清零。
   * BIT[5]-电流错误置 1，电流正常后清零。
   * BIT[6]-功率错误置 1，功率正常后清零。
   * BIT[7]-温度错误置 1，温度正常后清零。
   */
  delay(1000);  // 等待1000ms
}

void loop() {
  FSARM_JOINTS_STATE_T thetas;  // 关节角度
  arm.queryRawAngle(&thetas);   //查询更新原始舵机角度
  float& a1 = thetas.theta1;    //引用
  Serial.print("舵机0原始角度：");
  Serial.println(a1);
  // thetas.theta1 = 23.0;
  // thetas.theta2 = 5.9;
  // thetas.theta3 = 4.8;
  // thetas.theta4 = 3.6;
  //手臂速度很快，请注意安全

  // 设置
  arm.setRawAngle(thetas);  // 设置舵机旋转到特定的角度
  arm.wait();               // 等待舵机旋转到目标位置
  Serial.println(arm.servos[0].queryVoltage());
  Serial.println(arm.servos[0].queryStatus(), BIN);
  Serial.println(arm.servos[1].queryStatus(), BIN);
  Serial.println(arm.servos[2].queryStatus(), BIN);
  Serial.println(arm.servos[3].queryStatus(), BIN);
  Serial.println(arm.servos[4].queryStatus(), BIN);
  delay(5000);
  if (a1 > 133) {
    for (; a1 > -135; a1--) {
      arm.setRawAngle(thetas);  // 设置舵机旋转到特定的角度
      arm.wait();               // 等待舵机旋转到目标位置
      Serial.println(arm.servos[0].queryVoltage());
      Serial.println(arm.servos[0].queryStatus(), BIN);
      Serial.println(thetas.theta1);
      delay(10);  // 等待100ms
    }
  }

  else {
    for (; a1 < 135; a1++) {
      arm.setRawAngle(thetas);  // 设置舵机旋转到特定的角度
      arm.wait();               // 等待舵机旋转到目标位置
      Serial.println(arm.servos[0].queryVoltage());
      Serial.println(arm.servos[0].queryStatus(), BIN);
      Serial.println(thetas.theta1);
      delay(10);  // 等待100ms
    }
  }
}
//超过原始角度限制后舵机会失力一直显示执行中
//注意起始角度，超过绝对值160角度查询为零的情况。