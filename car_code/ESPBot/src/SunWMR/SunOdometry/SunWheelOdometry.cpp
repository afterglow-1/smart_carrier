/**
 * @brief Source:
 *
 */

#include "SunWheelOdometry.hpp"
//构造函数

WheelOdometry::WheelOdometry(Kinematics *kinematic) : _kinematic(kinematic)
{
    botPosition = {0.0, 0.0, 0.0};
}

//输入脉冲数 得到位姿
WheelOdometry::output WheelOdometry::getPositon_mm(int pulses1, int pulses2,
                                                   int pulses3, int pulses4)
{
    //脉冲数得到机器人局部坐标系下的速度
    WheelOdometry::output dPostion;
    //获得小车局部坐标系下车速
    Kinematics::velocities carVel =
        _kinematic->pulsesCalculateVelocities(pulses1, pulses2, pulses3, pulses4);
    //获得局部坐标系下的位移 单位mm
    dPostion.position_x = carVel.linear_x * TIMER_PERIOD;
    dPostion.position_y = carVel.linear_y * TIMER_PERIOD;
    dPostion.heading_theta = carVel.angular_z * TIMER_PERIOD;
    //得到全局坐标系的位置
    botPosition.heading_theta += dPostion.heading_theta;
    botPosition.position_x += kx * (cos(botPosition.heading_theta) * dPostion.position_x -
                                    sin(botPosition.heading_theta) * dPostion.position_y);
    botPosition.position_y += ky * (sin(botPosition.heading_theta) * dPostion.position_x +
                                    cos(botPosition.heading_theta) * dPostion.position_y);

    return botPosition;
}
//融合陀螺仪航向角的里程计
WheelOdometry::output WheelOdometry::getPositon_mm(int pulses1, int pulses2,
                                                   int pulses3, int pulses4, float imu_yaw)
{
    //脉冲数得到机器人局部坐标系下的速度
    WheelOdometry::output dPostion;
    Kinematics::velocities carVel =
        _kinematic->pulsesCalculateVelocities(pulses1, pulses2, pulses3, pulses4);
    //获得局部坐标系下的位移 单位mm
    dPostion.position_x = carVel.linear_x * TIMER_PERIOD;
    dPostion.position_y = carVel.linear_y * TIMER_PERIOD;

    //用陀螺仪返回角度代替运动学偏航角
    botPosition.heading_theta = imu_yaw; //不同库方向定义不同，IICDEV库角度与小车坐标系角度相反,请统一到小车坐标系
                                         //将局部坐标系下的位移变换到全局坐标系下并累加
    botPosition.position_x += kx * (cos(botPosition.heading_theta) * dPostion.position_x -
                                    sin(botPosition.heading_theta) * dPostion.position_y);
    botPosition.position_y += ky * (sin(botPosition.heading_theta) * dPostion.position_x +
                                    cos(botPosition.heading_theta) * dPostion.position_y);

    return botPosition;
}

void WheelOdometry::setPositon_mm(float real_position_x, float real_position_y, float real_heading_theta)
{
    botPosition.position_x = real_position_x;
    botPosition.position_y = real_position_y;
    botPosition.heading_theta = real_heading_theta;
}
void WheelOdometry::calibrationK(float k_x, float k_y, float k_theta)
{
    kx = k_x;
    ky = k_y;
    ktheta = k_theta;
}
