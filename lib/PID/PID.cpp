#include "PID.h"
PID Rot_PID;
PID MoveX_PID;
PID MoveY_PID;
PID FollowX_PID;
PID FollowY_PID;
// 用于初始化任意一个PID控制器
void PID::PID_Init(float p, float i, float d, float maxI, float maxOut)
{
    Kp = p;
    Ki = i;
    Kd = d;
    maxIntegral = maxI;
    maxOutput = maxOut;
    error = 0;
    lastError = 0;
    integral = 0;
    output = 0;
    lastDerivative = 0;
}
// 用于PID控制，输入参数为PID控制器、目标量与实际量
void PID::PID_Calc(float Target_Para, float Current_Para)
{
    // 更新数据
    lastError = error;             // 将旧error存起来
    error = Target_Para - Current_Para; // 计算新error
    if(this == &Rot_PID)
    {
        if (fabs(error) > M_PI) // 对临界点的偏差进行修正，即-180~180度突变的点
        {
            error = (error > 0) ? error - 2 * M_PI : error + 2 * M_PI;
        }
        // 计算积分
        if (fabs(error) < 0.5) // 0.5rad ≈ 28.6°
        {
            integral += (error) * (Ki);
            // 积分限幅
            //integral = constrain(integral, -maxIntegral, maxIntegral);
        }
        else
        {
            integral = 0; // 大误差时重置积分
        }
    }
    else
    {
        if (fabs(error) < 30)
        {
            integral += (error) * (Ki);
            // 积分限幅
            integral = constrain(integral, -maxIntegral, maxIntegral);
        }
        else
        {
            integral = 0; // 大误差时重置积分
        }
    }

    // 计算微分
    float derivative = (error - lastError);
    float filteredDerivative = 0.7 * lastDerivative + 0.3 * derivative;
    lastDerivative = filteredDerivative;
    // 计算比例
    float P_out = (error) * (Kp);
    // 计算输出
    output = P_out + (Kd) * filteredDerivative + integral;
    // 输出限幅
    //output = constrain(output, -maxOutput, maxOutput);
}