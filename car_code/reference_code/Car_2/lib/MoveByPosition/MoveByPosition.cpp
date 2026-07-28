#include "MoveByPosition.h"
// 电机对象实例化
float CurrentRad = 0;
float CurrentYaw = 0;
// 全局坐标，记录当前坐标，单位为mm
float Current_X = 0;
float Current_Y = 0;
// 记录绝对误差
float Error_mm = 0.0;
float Error_Rad = 0.0;
float Delta_Rad = 0.0;
// 定义运动状态
CARSTATE CarState = IDLE;
bool isRunning = false;
// 定义全局点位
/*
CAR_GOAL_POINT MoveSquence[] =
{
    {0, 0, 0, 50, 2, 200, 2},         // 0原点
    {0, 0, M_PI / 2, 50, 2, 200, 2},  // 1扫码
    {0, 0, M_PI, 50, 2, 200, 2},      // 2抓取
    {0, 0, M_PI / 2, 50, 2, 200, 2},  // 3路口旋转
    {0, 0, M_PI, 50, 2, 200, 2},      // 4粗加工区旋转
    {0, 0, M_PI / 2, 50, 2, 200, 2},  // 5粗加工中心
    {0, 0, 0, 50, 2, 200, 2},         // 6拐角1
    {0, 0, -M_PI / 2, 50, 2, 200, 2}, // 7存储区中心
    {0, 0, 0, 50, 2, 200, 2},         // 8拐角2
    {0, 0, M_PI, 50, 2, 200, 2}       // 9原点
}; // 每次移动的距离，基于局部坐标
*/

extern CAR_GOAL_POINT MoveSquence[];

/*
CAR_GOAL_POINT MoveSquence[] =
{
    {0, 0, 0, 400, 2, 200, 2},         // 0原点
    {1000, 0, M_PI / 2, 400, 2, 200, 2},  // 1扫码
    {1000, 1000, M_PI, 50, 2, 200, 2},      // 2抓取
    {0, 1000, M_PI / 2, 50, 2, 200, 2},  // 3路口旋转
};
*/
// 全局指针，用于遍历移动动作
int MoveIndex = 0;
// 关于PID闭环控制有关的阈值参数，速度计算，ClosedLoop_cal()
float ClosedLoop_mm = 0; // 进入PID闭环控制的阈值
float Slow_mm = 0;       // 闭环控制中线性减速的阈值d
// 用于间断运动
unsigned long lastMoveTime = 0;
const long MoveInterval = 0;
// 实例化电机
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);
AccelStepper stepper4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);
/// @brief 初始化电机、使能电机与复位电机
void Motor_Init() // 用于电机的复位操作
{
    // 使能引脚
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, 0); // 使能所有步进电机，低电平有效

    stepper1.setCurrentPosition(0); // 复位步进电机初始位置
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
}

/// @brief 设定运动的转速与加速时间
/// @param Rpm
/// @param Accel_Time
void Motor_Setup(float Rpm, float Accel_Time) // Rpm为转速 转/min，最大为3000rpm,Accel_Time为到达最大速度的时间，单位为s
{
    // 用于设置电机的转速、加速度等
    float MaxSpeed = (Pulse_Num * Rpm) / 60.0;
    float Acceleration = (Pulse_Num * Rpm) / (60.0 * Accel_Time);
    // 电机初始化操作
    stepper1.setMaxSpeed(MaxSpeed); // 设置1#电机最大速度，单位为脉冲数/s；
    stepper1.setAcceleration(Acceleration);

    stepper2.setMaxSpeed(MaxSpeed); // 设置2#电机最大速度，单位为脉冲数/s；
    stepper2.setAcceleration(Acceleration);

    stepper3.setMaxSpeed(MaxSpeed); // 设置3#电机最大速度，单位为脉冲数/s；
    stepper3.setAcceleration(Acceleration);

    stepper4.setMaxSpeed(MaxSpeed); // 设置4#电机最大速度，单位为脉冲数/s；
    stepper4.setAcceleration(Acceleration);
}
/// @brief 相对移动
/// @param X_Distance
/// @param Y_Distance
/// @param Rpm
/// @param Accel_Time
void MoveCar(float X_Distance, float Y_Distance, float Rpm, float Accel_Time) // x方向与y方向移动的距离，单位为mm；主要使用位置控制,阻塞式控制，防止开环步进时还没结束运动便执行下一个
{
    Motor_Setup(Rpm, Accel_Time);

    stepper1.move(motor1_CW * (long)((X_Distance - Y_Distance) * PulseNum_mm));
    stepper2.move(motor2_CW * (long)((X_Distance + Y_Distance) * PulseNum_mm));
    stepper3.move(motor3_CW * (long)((X_Distance + Y_Distance) * PulseNum_mm));
    stepper4.move(motor4_CW * (long)((X_Distance - Y_Distance) * PulseNum_mm));

    CarState = Moving;
}
/// @brief 用于全局坐标移动下的开环点位移动
/// @param X_Global_Distance
/// @param Y_Global_Distance
/// @param Rpm
/// @param Accel_Time
void Global_MoveCar(float X_Global_Distance, float Y_Global_Distance, float Rpm, float Accel_Time) // 全局坐标下x方向与y方向移动的距离，单位为mm；
{
    float X_Distance = X_Global_Distance * cos(CurrentRad) + Y_Global_Distance * sin(CurrentRad); // 换算到局部坐标
    float Y_Distance = Y_Global_Distance * cos(CurrentRad) - X_Global_Distance * sin(CurrentRad);
    MoveCar(X_Distance, Y_Distance, Rpm, Accel_Time);
}
/// @brief 用于全局坐标下的闭环点位移动
/// @param Target_X
/// @param Target_Y
/// @param Rpm
/// @param Accel_Time
void MoveCar_toTarget(float Target_X, float Target_Y, float Rpm, float Accel_Time)
{
    // 函数参数为目标坐标，转速以及加速度，加入PID反馈，适用于绝对坐标下的走点，坐标为全局坐标
    MoveX_PID.PID_Calc(Target_X, Current_X); // 获取X方向第一次的误差以及输出量
    MoveY_PID.PID_Calc(Target_Y, Current_Y); // 获取Y方向第一次的误差以及输出量
    // 动态调整速度：大误差时高速，小误差时低速
    float adaptiveRpm = Rpm;

    if (fabs(MoveX_PID.error) < Slow_mm && fabs(MoveY_PID.error) < Slow_mm)
    // 必须是逻辑与，因为只往一个方向走时，速度会始终限制在另一个方向的Rpm,并且防止过小引发的反向放大
    {
        adaptiveRpm = max(fabs(Rpm * (MoveX_PID.error / Slow_mm)),  // 选用max而不是min原因同上
                          fabs(Rpm * (MoveY_PID.error / Slow_mm))); // 线性减速
        adaptiveRpm = max(adaptiveRpm, (float)(5.0));
    }

    Global_MoveCar(MoveX_PID.output, MoveY_PID.output, adaptiveRpm, Accel_Time);
}
/// @brief 相对角度转动
/// @param Theta
/// @param Rpm
/// @param Accel_Time
void RotateCar(float Theta, float Rpm, float Accel_Time)
{
    // 原地自转角度，只需要简化成为每次旋转M_PI/2即可，并指定方向，以俯视逆时针为正
    Motor_Setup(Rpm, Accel_Time);

    stepper1.move(motor1_CW * (long)(-Theta * rotationFactor));
    stepper2.move(motor2_CW * (long)(Theta * rotationFactor));
    stepper3.move(motor3_CW * (long)(-Theta * rotationFactor));
    stepper4.move(motor4_CW * (long)(Theta * rotationFactor));

    CarState = Rotating;
}
/// @brief 用于全局坐标下的闭环角度调整
/// @param TargetRad
/// @param Rpm
/// @param Accel_Time
void RotateCar_toTarget(float TargetRad, float Rpm, float Accel_Time)
{
    // 用于原地自转，函数参数为当前姿态角与目标姿态角，转速以及加速度，加入PID反馈，适用于绝对坐标下的走点
    Rot_PID.PID_Calc(TargetRad, CurrentRad); // 更新误差以及输出量
    // 动态调整速度：大误差时高速，小误差时低速
    float adaptiveRpm = Rpm;
    float SlowDown_rad = 0.1;

    if (fabs(Rot_PID.error) < SlowDown_rad) // 0.1rad ≈ 6°
    {
        adaptiveRpm = fabs(Rpm * (Rot_PID.error / (SlowDown_rad))); // 线性减速
    }
    RotateCar(Rot_PID.output, adaptiveRpm, Accel_Time);
}
/// @brief 使能电机运转，即发送脉冲
void RunMotors()
{
    // 电机运转函数，发送脉冲
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
}
/// @brief 根据移动点位计算当前移动下的闭环阈值与线性减速阈值
void CloseLoop_Cal()
{
    float delta_x;
    float delta_y;
    if (MoveIndex != 0)
    {
        delta_x = fabs(MoveSquence[MoveIndex].x - MoveSquence[MoveIndex - 1].x);
        delta_y = fabs(MoveSquence[MoveIndex].y - MoveSquence[MoveIndex - 1].y);
    }
    else
    {
        delta_x = fabs(MoveSquence[0].x - MoveSquence[Point_Num - 1].x);
        delta_y = fabs(MoveSquence[0].y - MoveSquence[Point_Num - 1].y);
    }

    Slow_mm = max((int)max(delta_x, delta_y) / 20, 30);       // 防止小位移动时无法正常减速

}
/// @brief 总运动函数，用于根据运动状态与目标点位，先移动后旋转；支持线性减速，间隔运动
void RunCar()
{
    // 小车移动集成函数
    // 主要使用状态机实现不同状态的切换，全局变量MoveMent设定每次运动的情况，使用MoveIndex进行遍历
    // 用于获取当前绝对坐标偏差值，计算距离目标点的欧式距离
    // 不能使用定时器实时获取小车状态，因为会导致误差获取频率过低
    Error_mm = sqrt(pow(MoveSquence[MoveIndex].x - Current_X, 2) + pow(MoveSquence[MoveIndex].y - Current_Y, 2));
    Delta_Rad = (MoveSquence[MoveIndex].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }
    isRunning = (stepper1.isRunning() ||
                 stepper2.isRunning() ||
                 stepper3.isRunning() ||
                 stepper4.isRunning());
    RunMotors();      // 循环使能电机
    switch (CarState) // 状态转移,计时放在旋转完与下一次移动之间
    {
    case IDLE:
        if (millis() - lastMoveTime > MoveInterval)
        {
            MoveCar_toTarget(MoveSquence[MoveIndex].x,
                             MoveSquence[MoveIndex].y,
                             MoveSquence[MoveIndex].Move_Rpm,
                             MoveSquence[MoveIndex].Move_Accel_Time);
            CloseLoop_Cal(); // 计算本次运动的相关运动参数
        }
        break;
    case Moving:
        if (!isRunning && fabs(Error_mm) <= MaxError_Move)
        {
            CarState = Move_Complete;
        }
        else if (fabs(Error_mm) > MaxError_Move)
        {
            MoveCar_toTarget(MoveSquence[MoveIndex].x,
                             MoveSquence[MoveIndex].y,
                             MoveSquence[MoveIndex].Move_Rpm,
                             MoveSquence[MoveIndex].Move_Accel_Time);
        }
        break;
    case Move_Complete:
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        RotateCar_toTarget(MoveSquence[MoveIndex].z, MoveSquence[MoveIndex].Rot_Rpm, MoveSquence[MoveIndex].Rot_Accel_Time);
        break;
    case Rotating:
        if (!isRunning && fabs(Error_Rad) <= MaxError_Rot)
        {
            CarState = Rotate_Complete;
        }
        else
        {
            RotateCar_toTarget(MoveSquence[MoveIndex].z, MoveSquence[MoveIndex].Rot_Rpm, MoveSquence[MoveIndex].Rot_Accel_Time);
        }
        break;
    case Rotate_Complete:
        MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        // 表示已经调整到位，重新初始化PID控制器
        // MoveIndex++;
        // if (MoveIndex == Point_Num)
        //     MoveIndex = 0; // 溢出，回到循环开始
        // CarState = IDLE;***************这部分操作改由操作机器人实现
        lastMoveTime = millis();
        break;
    }
}

/// @brief 使能速度控制的电机，使电机处于匀速运动
void RunMotors_Speed()
{
    // 使能电机
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
}
/// @brief 采用对数自适应调整比例系数
/// @param Kp 基础比例系数
/// @param Rpm 设定最大速度，测试为200
/// @param Accel_time 设定加速时间，测试为2s
void Follow_bySpeed(float Kp, float Rpm, float Accel_time, float Move_X_Grab, float Move_Y_Grab)
{
    // 用于设置电机的转速、加速度等
    // Motor_Setup(Rpm, Accel_time);
    // 发送脉冲驱动电机
    RunMotors_Speed();
    // 自适应比例环
    float Kp_x = Kp + log10(fabs(Move_X_Grab) + 1);
    float Kp_y = Kp + log10(fabs(Move_Y_Grab) + 1);

    float Global_vx = Kp_x * Move_X_Grab;
    float Global_vy = Kp_y * Move_Y_Grab;
    float Global_vz = 0;

    // 计算当前目标移速
    float targetStepper1Speed = motor1_CW * (Global_vx - Global_vy - Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    float targetStepper2Speed = motor2_CW * (Global_vx + Global_vy + Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    float targetStepper3Speed = motor3_CW * (Global_vx + Global_vy - Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    float targetStepper4Speed = motor4_CW * (Global_vx - Global_vy + Global_vz * (LR_WHEELS_DISTANCE + FR_WHEELS_DISTANCE) / 2) * 60.0 / C_Wheel;
    // 电机转速设置
    stepper1.setSpeed(Pulse_Num * targetStepper1Speed );
    stepper2.setSpeed(Pulse_Num * targetStepper2Speed );
    stepper3.setSpeed(Pulse_Num * targetStepper3Speed );
    stepper4.setSpeed(Pulse_Num * targetStepper4Speed );
}


/// @brief 用于OPS失效下的开环跑点
void RunCar_open()
{  
    digitalWrite(LED,1);
    RunMotors();//循环使能电机  
    if (MoveIndex == 0) MoveIndex = 1;// 跳过原点的点位
    Delta_Rad = (MoveSquence[MoveIndex].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }  

    switch (CarState)//状态转移,计时放在旋转完与下一次移动之间
    {
        case IDLE:
            if(millis() - lastMoveTime > MoveInterval)//用于设定定时运动
            {
                Global_MoveCar( MoveSquence[MoveIndex].x - MoveSquence[MoveIndex-1].x,
                                MoveSquence[MoveIndex].y - MoveSquence[MoveIndex-1].y,
                                MoveSquence[MoveIndex].Move_Rpm,
                                MoveSquence[MoveIndex].Move_Accel_Time
                                );
            }
            break;
        case Moving:
            if( !stepper1.isRunning()&& 
                !stepper2.isRunning()&& 
                !stepper3.isRunning()&& 
                !stepper4.isRunning())
            {
                CarState = Move_Complete;
            }
            break;
        case Move_Complete:
            Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
            RotateCar(MoveSquence[MoveIndex].z,MoveSquence[MoveIndex].Rot_Rpm,MoveSquence[MoveIndex].Rot_Accel_Time);
            break;
        case Rotating:
            if( !stepper1.isRunning()&& 
                !stepper2.isRunning()&& 
                !stepper3.isRunning()&& 
                !stepper4.isRunning()&& 
                fabs(Error_Rad) <= MaxError_Rot)
            {
                CarState = Rotate_Complete;
            }
            else
            {
                RotateCar_toTarget(MoveSquence[MoveIndex].z, MoveSquence[MoveIndex].Rot_Rpm, MoveSquence[MoveIndex].Rot_Accel_Time);
            }
            break;
        case Rotate_Complete:
            Current_X = MoveSquence[MoveIndex].x;
            Current_Y = MoveSquence[MoveIndex].y;
            // MoveIndex++;
            // if (MoveIndex == Point_Num)
            //     MoveIndex = 0; // 溢出，回到循环开始
            // CarState = IDLE;
            lastMoveTime = millis();
            break;
    }
}


/// @brief 用于同时平移与旋转的运动，并无拐弯
/// @param Target_X 
/// @param Target_Y 
/// @param TargetRad 
/// @param Rpm 
/// @param Accel_Time 
void RunToTarget(float Target_X, float Target_Y, float TargetRad,float Rpm, float Accel_Time)
{
    MoveX_PID.PID_Calc(Target_X, Current_X); // 获取X方向第一次的误差以及输出量
    MoveY_PID.PID_Calc(Target_Y, Current_Y); // 获取Y方向第一次的误差以及输出量
    Rot_PID.PID_Calc(TargetRad, CurrentRad); // 获取z方向第一次的误差以及输出量
        
    float adaptiveRpm = Rpm;
    if(TargetRad != MoveSquence[MoveIndex-1].z)
    {
        // 若有旋转，则旋转时开始线性减速
        float SlowDown_rad = 0.5;
        if (fabs(Rot_PID.error) < SlowDown_rad) // 0.1rad ≈ 6°
        {
            adaptiveRpm = fabs(Rpm * (Rot_PID.error / (SlowDown_rad))); // 线性减速
        }
    }
    else
    {
        float Slow_mm = 100;
        // 若无旋转，则根据平移开始线性减速
        if (fabs(MoveX_PID.error) < Slow_mm && fabs(MoveY_PID.error) < Slow_mm)
        // 必须是逻辑与，因为只往一个方向走时，速度会始终限制在另一个方向的Rpm,并且防止过小引发的反向放大
        {
            adaptiveRpm = max(fabs(Rpm * (MoveX_PID.error / Slow_mm)),  // 选用max而不是min原因同上
                            fabs(Rpm * (MoveY_PID.error / Slow_mm)));   // 线性减速
            adaptiveRpm = max(adaptiveRpm, (float)(5.0));
        }
    }


    float X_Distance = MoveX_PID.output * cos(CurrentRad) + MoveY_PID.output * sin(CurrentRad); // 换算到局部坐标
    float Y_Distance = MoveY_PID.output * cos(CurrentRad) - MoveX_PID.output * sin(CurrentRad);

    Motor_Setup(adaptiveRpm, Accel_Time);

    stepper1.move(motor1_CW * (long)((X_Distance - Y_Distance) * PulseNum_mm - Rot_PID.output * rotationFactor));
    stepper2.move(motor2_CW * (long)((X_Distance + Y_Distance) * PulseNum_mm + Rot_PID.output * rotationFactor));
    stepper3.move(motor3_CW * (long)((X_Distance + Y_Distance) * PulseNum_mm - Rot_PID.output * rotationFactor));
    stepper4.move(motor4_CW * (long)((X_Distance - Y_Distance) * PulseNum_mm + Rot_PID.output * rotationFactor));

    CarState = Moving;
    
}
void RunCar_MaR()
{
    // 小车移动集成函数
    // 移动方式为同时旋转与平移
    // 状态改为三个状态
    Error_mm = sqrt(pow(MoveSquence[MoveIndex].x - Current_X, 2) + pow(MoveSquence[MoveIndex].y - Current_Y, 2));
    Delta_Rad = (MoveSquence[MoveIndex].z - CurrentRad);
    if (fabs(Delta_Rad) > M_PI) // 对临界点的偏差进行修正，即180~-180度突变的点
    {
        Error_Rad = (Delta_Rad > 0) ? Delta_Rad - 2 * M_PI : Delta_Rad + 2 * M_PI; // 表示控制量
    }
    else
    {
        Error_Rad = Delta_Rad;
    }
    isRunning = (stepper1.isRunning() ||
                 stepper2.isRunning() ||
                 stepper3.isRunning() ||
                 stepper4.isRunning());

    RunMotors();      // 循环使能电机
    switch (CarState) // 状态转移,计时放在旋转完与下一次移动之间
    {
    case IDLE:
        if (millis() - lastMoveTime > 0)
        {
            if(MoveIndex == 4)
            {
                Global_MoveCar( MoveSquence[MoveIndex].x - MoveSquence[MoveIndex-1].x,
                                MoveSquence[MoveIndex].y - MoveSquence[MoveIndex-1].y,
                                MoveSquence[MoveIndex].Move_Rpm,
                                MoveSquence[MoveIndex].Move_Accel_Time
                );
            }
            else
            {
                RunToTarget(MoveSquence[MoveIndex].x,
                            MoveSquence[MoveIndex].y,
                            MoveSquence[MoveIndex].z,
                            MoveSquence[MoveIndex].Move_Rpm,
                            MoveSquence[MoveIndex].Move_Accel_Time);
            }
        }
        break;
    case Moving:
        if (MoveIndex == 4  && !isRunning) // 让该点运动到末端时直接进入下一个点位
        {
            CarState = Move_Complete;
        }
        else
        {
            if (fabs(Error_mm) <= MaxError_Move && fabs(Error_Rad) <= MaxError_Rot)
            {
                CarState = Move_Complete;
            }
            else if(fabs(Error_mm) > MaxError_Move || fabs(Error_Rad) > MaxError_Rot)
            {
                RunToTarget(MoveSquence[MoveIndex].x,
                        MoveSquence[MoveIndex].y,
                        MoveSquence[MoveIndex].z,
                        MoveSquence[MoveIndex].Move_Rpm,
                        MoveSquence[MoveIndex].Move_Accel_Time);   
            }
        }
        break;
    case Move_Complete:
        MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
        Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut); // 表示已经调整到位，重新初始化PID控制器
        MoveIndex++;
        if (MoveIndex == Point_Num)
        {
             MoveIndex = 0; // 溢出，回到循环开始
         }
        CarState = IDLE;
        lastMoveTime = millis();
        break;
    }
}