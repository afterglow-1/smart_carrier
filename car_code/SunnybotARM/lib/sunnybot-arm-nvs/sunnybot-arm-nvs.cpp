/*
 * FashionStar 五自由度机械臂SDK (Arduino)
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */

#include "sunnybot-arm-nvs.h"

ARMNVS::ARMNVS() {}

// 读取NVS标定数据
bool ARMNVS::readARMNVS() {
  // 只读模式打开或创建ARMNVS命名空间
  prefs.begin("ARMNVS", RO_MODE);
  bool armInit = prefs.isKey(
      "chipid");  // Test for the existence of the "already initialized" key.
  if (armInit == false) {
    prefs.end();
    return false;

  } else {
    nvsData.chipid = prefs.getULong64("chipid", 0);
    nvsData.count = prefs.getUInt("count", 0);
    nvsData.theta1 = prefs.getFloat("theta1", 0);
    nvsData.theta2 = prefs.getFloat("theta2", 0);
    nvsData.theta3 = prefs.getFloat("theta3", 0);
    nvsData.theta4 = prefs.getFloat("theta4", 0);
    nvsData.gripperOpen = prefs.getFloat("gripperOpen", 0);
    nvsData.turn1 = prefs.getBool("turn1", 0);
    nvsData.turn2 = prefs.getBool("turn2", 0);
    nvsData.turn3 = prefs.getBool("turn3", 0);
    nvsData.turn4 = prefs.getBool("turn4", 0);

    prefs.end();
    return true;
  }

}

  void ARMNVS::printARMNVS() {
    PRINT_UART.print("芯片ID为：");
    PRINT_UART.println(nvsData.chipid);
    PRINT_UART.print("标定数据为：");
    PRINT_UART.print(nvsData.theta1);
    PRINT_UART.print(",");
    PRINT_UART.print(nvsData.theta2);
    PRINT_UART.print(",");
    PRINT_UART.print(nvsData.theta3);
    PRINT_UART.print(",");
    PRINT_UART.print(nvsData.theta4);
    PRINT_UART.print(",");
    PRINT_UART.print(nvsData.gripperOpen);
    PRINT_UART.print(",");
    PRINT_UART.print(nvsData.turn1);
    PRINT_UART.print(",");
    PRINT_UART.print(nvsData.turn2);
    PRINT_UART.print(",");
    PRINT_UART.print(nvsData.turn3);
    PRINT_UART.print(",");
    PRINT_UART.println(nvsData.turn4);
    prefs.begin("ARMNVS", RO_MODE);
    PRINT_UART.print("NVS剩余空间：");
    PRINT_UART.println(prefs.freeEntries());
    PRINT_UART.printf("系统已角度标定 %u 次\n", prefs.getUInt("count", 0));
    prefs.end();  // 关闭当前命名空间
  }

// 保存标定数据到NVS bool* sameTurn等价
void ARMNVS::writeARMNVS(FSARM_JOINTS_STATE_T thetas, bool sameTurn[4], float gripperOpen) {
  prefs.begin("ARMNVS", RW_MODE);
  // The chip ID is essentially its MAC address(length: 6 bytes).
  prefs.putULong64("chipid", ESP.getEfuseMac());
  prefs.putFloat("theta1", thetas.theta1);
  prefs.putFloat("theta2", thetas.theta2);
  prefs.putFloat("theta3", thetas.theta3);
  prefs.putFloat("theta4", thetas.theta4);
  prefs.putFloat("gripperOpen", gripperOpen);
  prefs.putBool("turn1", sameTurn[0]);
  prefs.putBool("turn2", sameTurn[1]);
  prefs.putBool("turn3", sameTurn[2]);
  prefs.putBool("turn4", sameTurn[3]);
  prefs.putUInt("count", prefs.getUInt("count", 0) +
                             1);  // 将数据保存到当前命名空间的"count"键中
  prefs.end();                    // 关闭当前命名空间
}
