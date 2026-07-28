/*
 *
 程序名称：sunarm-set-angle
 设置机械臂关节的角度
 todo：添加手爪校准
 */
#include <Arduino.h>
#include "FashionStar_Arm5DoF.h"
#include "sunnybot-arm-nvs.h"
#include <U8g2lib.h>
// 调试串口的配置
#if defined(ARDUINO_AVR_UNO)
#include <SoftwareSerial.h>
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX);  // 创建软串口
#define DEBUG_SERIAL softSerial
#define DEBUG_SERIAL_BAUDRATE 4800
#elif defined(ARDUINO_AVR_MEGA2560)
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200
#elif defined(ARDUINO_ARCH_ESP32)
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200
#endif
FSARM_ARM5DoF arm;  //机械臂对象
ARMNVS armnvs;
void setup() {
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);  // 初始化串口的波特率
  arm.init();                                 //机械臂初始化
  arm.gripperOpen();                        //测试手爪开启
  if (armnvs.readARMNVS()) {
    armnvs.printARMNVS();  //打印原有标定数据
    //使用标定数据进行标定
    arm.calibration(armnvs.nvsData.theta1, armnvs.nvsData.turn1,
                    armnvs.nvsData.theta2, armnvs.nvsData.turn2,
                    armnvs.nvsData.theta3, armnvs.nvsData.turn3,
                    armnvs.nvsData.theta4, armnvs.nvsData.turn4);
  }
  arm.setAngle(0, 5.0);  //为了体现回零确实有动作
  arm.wait();
  arm.setSpeed(150);
  arm.home();  //回参考点位置
}

void loop() {
  // 如果提供了初始化列表，那么可以在数组定义中省略数组长度，数组长度由初始化器列表中最后一个数组元素的索引值决定
  FSARM_JOINTS_STATE_T thetas[] = {{45.0, -130.0, 90.0, 60.0},
                                   {-90.0, -130.0, 120.0, 30.0},
                                   {0.0, -130.0, 120.0, 30.0},
                                   {0.0, -90.0, 120.0, 30.0}};
  bool gripperActions[] = {0, 0, 1, 0  };

  int actionsLenth = (sizeof(thetas) / sizeof(thetas[0]));
  //获得动作组动作数量
  DEBUG_SERIAL.print("动作组动作数量为：");
  DEBUG_SERIAL.println(actionsLenth);
  for (size_t i = 0; i < actionsLenth; i++) {
    arm.setAngle(thetas[i]);  // 设置舵机旋转到特定的角度
    arm.wait();               // 等待舵机旋转到目标位置
    delay(1000);              // 等待1s
    if (gripperActions[i]) {
      arm.gripperOpen();
    } else {
      arm.gripperClose();
    }
  }
}