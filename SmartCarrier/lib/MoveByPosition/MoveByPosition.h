/**
  *****************************************************************************
  * @file               MoveByPosition.h
  * @author             邹子睿
  * @version            v1.0
  * @date               2025/6/24
  * @environment        STM32H7 (Arduino Framework)
  * @brief              Header file for functions of Moving
  *****************************************************************************
**/
#ifndef MoveByPosition_h
#define MoveByPosition_h

#include <Arduino.h>
#include "AccelStepper.h"
#include "PID.h"
// 定义步进电机引脚
#define LED PA15
#define enPin PE13 // 共同的使能引脚
#define motorInterfaceType 1
#define dirPin1 PD6
#define stepPin1 PD4
#define dirPin2 PE9
#define stepPin2 PE11
#define dirPin3 PD14
#define stepPin3 PD15
#define dirPin4 PC3_C
#define stepPin4 PA1

// 步进电机参数定义
#define D_Wheel 100                       // 车轮直径mm
#define Step_Angle 1.8                    // 步距角°
#define Step_Num (360 / Step_Angle)       // 全步进一圈的步数，即转一圈走多少个步距角
#define MicroStep 32                      // 细分步数，即多少个脉冲走一个步距角,越大移速越慢，低频振动越小
#define Pulse_Num (Step_Num * MicroStep)  // 当前细分步数下，走一圈的脉冲个数
#define LR_WHEELS_DISTANCE 195            // 左右轴距（mm）
#define FR_WHEELS_DISTANCE 185            // 前后长度（mm）
#define MAX_Rpm 3000                      // 最大转速
#define C_Wheel (M_PI * D_Wheel)          // 车轮转一圈的移动距离,车轮周长(mm)
#define PulseNum_mm (Pulse_Num / C_Wheel) // 个数/mm，即走每mm需要多少个脉冲个数
#define PulseNum_deg (Pulse_Num / 360)    // 个数/°，即走每°需要多少个脉冲个数
const float rotationFactor = (LR_WHEELS_DISTANCE / 2.0 + FR_WHEELS_DISTANCE / 2.0) * PulseNum_mm;

// 闭环运动相关参数说明
#define MaxError_Rot 0.01 // 用于陀螺仪修正角度的阈值，单位为rad
#define MaxError_Move 2   // 用于修正位移的阈值，单位为mm

//电机运动方向
#define motor1_CW (-1)
#define motor2_CW (1)
#define motor3_CW (-1)
#define motor4_CW (1)
// 全局坐标，记录当前偏航角，单位为rad与°，统一以逆时针为正，且范围转换为180~-180
extern float CurrentRad;
extern float CurrentYaw;
// 全局坐标，记录当前坐标，单位为mm
extern float Current_X;
extern float Current_Y;
// 记录绝对误差
extern float Error_mm;
extern float Error_Rad; // 用于获取当前处理后的角度误差值
extern float Delta_Rad; // 用于获取当前角度偏差值，范围为180~-180°
// 定义小车运动状态
enum CARSTATE
{
    IDLE = 1,
    Moving,
    Move_Complete, // 指平移动作完成，准备进入旋转状态
    Rotating,
    Rotate_Complete
};
extern CARSTATE CarState;
extern bool isRunning; // 电机是否在运动
// 点位结构体定义
typedef struct
{
    float x;
    float y;
    float z;
    float Move_Rpm = 400;
    float Move_Accel_Time = 1;
    float Rot_Rpm = 400;
    float Rot_Accel_Time = 1;
} CAR_GOAL_POINT;
extern CAR_GOAL_POINT MoveSquence[];
#define Point_Num 10
// 全局指针，用于遍历移动动作
extern int MoveIndex;
// 关于PID闭环控制有关的阈值参数，速度计算，ClosedLoop_cal()
extern float ClosedLoop_mm; // 进入PID闭环控制的阈值
extern float Slow_mm;       // 闭环控制中线性减速的阈值
// 用于间断运动
extern unsigned long lastMoveTime;
extern const long MoveInterval;
// 实例化电机
extern AccelStepper stepper1;
extern AccelStepper stepper2;
extern AccelStepper stepper3;
extern AccelStepper stepper4;
// 主要函数
void Motor_Init();
void Motor_Setup(float Rpm, float Accel_Time);
void MoveCar(float X_Distance, float Y_Distance, float Rpm, float Accel_Time);
void Global_MoveCar(float X_Global_Distance, float Y_Global_Distance, float Rpm, float Accel_Time);
void MoveCar_toTarget(float Target_X, float Target_Y, float Rpm, float Accel_Time);
void RotateCar(float Theta, float Rpm, float Accel_Time);
void RotateCar_toTarget(float TargetRad, float Rpm, float Accel_Time);
void RunMotors();
void CloseLoop_Cal();
void RunCar();
void RunMotors_Speed();
void Follow_bySpeed(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab);
void RunCar_open();
void RunToTarget(float Target_X, float Target_Y, float TargetRad,float Rpm, float Accel_Time);
void RunCar_MaR();
#endif