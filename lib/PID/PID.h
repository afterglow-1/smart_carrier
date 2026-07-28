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

    // PID控制器初始化函数
    void PID_Init(float p, float i, float d, float maxI, float maxOut);

    // PID控制器计算函数
    void PID_Calc(float Target_Para, float Current_Para);
};

// 定义关于自转的PID控制器
#define Rot_PID_Kp 1  // 增加比例增益以加快响应
#define Rot_PID_Ki 0.0 // 减小积分增益以减少超调和振荡
#define Rot_PID_Kd 1.8  // 增加微分增益以抑制振荡
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

// 定义关于跟随的PID控制器
#define Follow_PID_Kp 0.5
#define Follow_PID_Ki 0
#define Follow_PID_Kd 1.5
#define Follow_PID_MItg 100  // 单位为mm
#define Follow_PID_MOut 2000 // 单位为mm
extern PID FollowX_PID;
extern PID FollowY_PID;
#endif