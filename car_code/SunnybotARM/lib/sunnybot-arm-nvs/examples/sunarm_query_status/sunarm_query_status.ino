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
  Serial.print("舵机1原始角度：");
  Serial.println(thetas.theta2);
    Serial.print("舵机2原始角度：");
  Serial.println(thetas.theta3);
     Serial.print("舵机3原始角度：");
  Serial.println(thetas.theta4);


  // 查询状态
  Serial.print("舵机0电压：");
  Serial.print(arm.servos[0].queryVoltage());
  Serial.print("舵机0状态：");
  Serial.println(arm.servos[0].queryStatus(), BIN);
  Serial.print("舵机1电压：");
  Serial.print(arm.servos[1].queryVoltage());
  Serial.print("舵机1状态：");
  Serial.println(arm.servos[1].queryStatus(), BIN);
  Serial.print("舵机2电压：");
  Serial.print(arm.servos[2].queryVoltage());
  Serial.print("舵机2状态：");
  Serial.println(arm.servos[2].queryStatus(), BIN);
  Serial.print("舵机3电压：");
  Serial.print(arm.servos[3].queryVoltage());
  Serial.print("舵机3状态：");
  Serial.println(arm.servos[3].queryStatus(), BIN);
  Serial.print("舵机4电压：");
  Serial.print(arm.servos[4].queryVoltage());
  Serial.print("舵机4状态：");
  Serial.println(arm.servos[4].queryStatus(), BIN);
  delay(1000);  // 等待1000ms
}
//串口舵机使用注意事项:
//超过原始角度限制(-135,+135)后舵机会失力,状态会一直显示执行中
//注意起始角度，绝对值超过160度会出现查询为零的情况。
//手爪功率超出后会报功率错误
//报状态错误后，需要重新上电才能再次运行