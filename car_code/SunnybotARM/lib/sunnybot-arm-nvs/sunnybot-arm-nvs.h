#ifndef _SUNNYBOT_ARM_NVS_H
#define _SUNNYBOT_ARM_NVS_H
/*
 * sunnybot 机械臂 NVS数据保存
 * --------------------------
 * 作者: Sunny
 * 邮箱:
 * 更新时间: 2022/05/12
 * 20210827 增加带参数阻尼模式
 */
#include <Arduino.h>
#include <Preferences.h>  //非易失性存储 (NVS) 库主要用于在 flash 中存储键值格式的数据。
#include "FashionStar_Arm5DoF.h"  //

typedef struct {
  uint64_t chipid;   
  float theta1;
  float theta2;
  float theta3;
  float theta4;
  float gripperOpen;
  bool turn1;
  bool turn2;
  bool turn3;
  bool turn4;
  uint32_t count;
} NVS_DATA;


//extern FSARM_JOINTS_STATE_T thetas; //声明而非定义
#define RW_MODE false
#define RO_MODE true
#define PRINT_UART Serial

class ARMNVS {
 public:
  ARMNVS();
  bool readARMNVS();
  void printARMNVS();
  void writeARMNVS(FSARM_JOINTS_STATE_T thetas,bool doesSameTurn[4],float gripperOpen);

  // NVS
  Preferences prefs;
  NVS_DATA nvsData;
 private:
};

#endif
