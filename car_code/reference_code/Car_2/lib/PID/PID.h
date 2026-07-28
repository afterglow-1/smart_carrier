/**
  *****************************************************************************
  * @file               PID.h
  * @author             邹子睿
  * @version            v1.0
  * @date               2025/6/24
  * @environment        STM32H7 (Arduino Framework)
  * @brief              Header file for functions of PID
  *****************************************************************************
**/
#ifndef PID_h
#define PID_h
#include "Arduino.h"
// PID控制器结构体定义
/*
typedef struct
{
    float Kp, Ki, Kd;            // 三个系数
    float error, lastError;      // 误差、上次误差
    float integral, maxIntegral; // 积分、积分限幅
    float lastDerivative;
    float output, maxOutput; // 输出、输出限幅
} PID;
*/
// 定义PID控制器类
class PID
{
public:    
    float Kp, Ki, Kd;            // 三个系数
    float error, lastError;      // 误差、上次误差
    float integral, maxIntegral; // 积分、积分限幅
    float lastDerivative;
    float output, maxOutput; // 输出、输出限幅
    // PID控制器error清空
    void PID_Init();
    // PID控制器初始化函数
    void PID_Init(float p, float i, float d, float maxI, float maxOut);

    // PID控制器计算函数
    void PID_Calc(float Target_Para, float Current_Para);
};

// 定义关于自转的PID控制器
#define Rot_PID_Kp 1.8  // 增加比例增益以加快响应
#define Rot_PID_Ki 0.05 // 减小积分增益以减少超调和振荡
#define Rot_PID_Kd 1.0  // 增加微分增益以抑制振荡
#define Rot_PID_MItg MaxError_Rot
#define Rot_PID_MOut M_PI
extern PID Rot_PID;
// 定义关于移动的PID控制器
#define Move_PID_Kp 0.8
#define Move_PID_Ki 0
#define Move_PID_Kd 1.8
#define Move_PID_MItg 100  // 单位为mm
#define Move_PID_MOut 2000 // 单位为mm
extern PID MoveX_PID;
extern PID MoveY_PID;

// 定义关于机械臂的PID控制器
#define ARM_PID_Kp 0.2
#define ARM_PID_Ki 0.000
#define ARM_PID_Kd 0.75
#define ARM_PID_MItg 1500  // 单位为0.1mm
#define ARM_PID_MOut 1500 // 单位为0.1mm
extern PID ARM_PID;

#endif