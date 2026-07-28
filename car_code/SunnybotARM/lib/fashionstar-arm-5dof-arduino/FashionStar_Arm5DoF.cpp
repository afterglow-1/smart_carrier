/*
 * FashionStar 五自由度机械臂SDK (Arduino)
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */

#include "FashionStar_Arm5DoF.h"

FSARM_ARM5DoF::FSARM_ARM5DoF() {}

// 初始化
void FSARM_ARM5DoF::init() {
  this->protocol.init(115200);  // 初始化通信协议
  initServos();                 // 初始化所有的舵机
  calibration();                // 机械臂标定
  setAngleRange();              // 设置舵机角度范围
  setSpeed(30);                // 初始化舵机的转速
  gripperClose();               // 爪子闭合
}

// 初始化舵机
void FSARM_ARM5DoF::initServos() {
  // 本体舵机初始化
  for (uint8_t sidx = 0; sidx < FSARM_SERVO_NUM; sidx++) {
    this->servos[sidx].init(sidx, &this->protocol);
    //添加1ms延时 Sunny add
    //delay(1);
  }
  // 自适应夹爪初始化
  this->gripper_servo.init(GRIPPER_SERVO_ID, &this->protocol);
  this->gripper.init(&this->gripper_servo, GRIPPER_SERVO_ANGLE_OPEN,
                     GRIPPER_SERVO_ANGLE_CLOSE);
}

// 关节标定
void FSARM_ARM5DoF::calibration() {
  this->servos[FSARM_JOINT1].calibration(FSARM_JOINT1_P90, 90.0,
                                         FSARM_JOINT1_N90, -90.0);
  this->servos[FSARM_JOINT2].calibration(FSARM_JOINT2_P0, 0.0, FSARM_JOINT2_N90,
                                         -90.0);
  this->servos[FSARM_JOINT3].calibration(FSARM_JOINT3_P90, 90.0,
                                         FSARM_JOINT3_N90, -90.0);
  this->servos[FSARM_JOINT4].calibration(FSARM_JOINT4_P90, 90.0,
                                         FSARM_JOINT4_N90, -90.0);
 
}

  // 关节标定简化版 sunny add 
  //参数 各关节竖直安装位的角度 旋转方向 相同1 相反0
  void FSARM_ARM5DoF::calibration(float joint1_P0,bool direction1,float joint2_N90,bool direction2,float joint3_P0,bool direction3,float joint4_P0,bool direction4)
  {
  this->servos[FSARM_JOINT1].calibration(direction1?1:-1, joint1_P0);
  //y=kx+b  b=y-kx  y=joint2_N90 k=direction2?1:-1  x=-90
this->servos[FSARM_JOINT2].calibration(direction2?1:-1, (direction2?1:-1)*90+joint2_N90);
  this->servos[FSARM_JOINT3].calibration(direction3?1:-1, joint3_P0);
    this->servos[FSARM_JOINT4].calibration(direction4?1:-1, joint4_P0);
  }

// 设置舵机的角度范围
void FSARM_ARM5DoF::setAngleRange() {
  this->servos[FSARM_JOINT1].setAngleRange(FSARM_JOINT1_MIN, FSARM_JOINT1_MAX);
  this->servos[FSARM_JOINT2].setAngleRange(FSARM_JOINT2_MIN, FSARM_JOINT2_MAX);
  this->servos[FSARM_JOINT3].setAngleRange(FSARM_JOINT3_MIN, FSARM_JOINT3_MAX);
  this->servos[FSARM_JOINT4].setAngleRange(FSARM_JOINT4_MIN, FSARM_JOINT4_MAX);

}

// 开启扭矩
void FSARM_ARM5DoF::setTorque(bool enable) {
  for (uint8_t sidx = 0; sidx < FSARM_SERVO_NUM; sidx++) {
    this->servos[sidx].setTorque(enable);
  }
}
// 设置阻尼模式
void FSARM_ARM5DoF::setDamping() {
  for (uint8_t sidx = 0; sidx < FSARM_SERVO_NUM; sidx++) {
    this->servos[sidx].setDamping(1000);
  }

  this->gripper_servo.setDamping(1000);
}
// 设置阻尼模式，带参数
void FSARM_ARM5DoF::setDamping(int damp) {
  for (uint8_t sidx = 0; sidx < FSARM_SERVO_NUM; sidx++) {
    this->servos[sidx].setDamping(damp);
  }

  this->gripper_servo.setDamping(damp);
}

// 设置所有舵机的转速
void FSARM_ARM5DoF::setSpeed(FSUS_SERVO_SPEED_T speed) {
  for (uint8_t sidx = 0; sidx < FSARM_SERVO_NUM; sidx++) {
    if (sidx == FSARM_JOINT4) {
      this->servos[sidx].setSpeed(2 * speed);
    } else {
      this->servos[sidx].setSpeed(speed);
    }
  }
}

// 读取舵机原始角度
void FSARM_ARM5DoF::queryRawAngle(FSARM_JOINTS_STATE_T* thetas) {
  thetas->theta1 = this->servos[FSARM_JOINT1].queryRawAngle();
  thetas->theta2 = this->servos[FSARM_JOINT2].queryRawAngle();
  thetas->theta3 = this->servos[FSARM_JOINT3].queryRawAngle();
  thetas->theta4 = this->servos[FSARM_JOINT4].queryRawAngle();
}

// 读取角度
// 查询所有舵机的角度并填充在thetas里面
void FSARM_ARM5DoF::queryAngle(FSARM_JOINTS_STATE_T* thetas) {
  thetas->theta1 = this->servos[FSARM_JOINT1].queryAngle();
  thetas->theta2 = this->servos[FSARM_JOINT2].queryAngle();
  thetas->theta3 = this->servos[FSARM_JOINT3].queryAngle();
  thetas->theta4 = this->servos[FSARM_JOINT4].queryAngle();
  // thetas->gripper = this->servos[FSARM_GRIPPER].queryAngle();
}

// 设置舵机的原始角度
void FSARM_ARM5DoF::setRawAngle(FSARM_JOINTS_STATE_T thetas) {
  this->servos[FSARM_JOINT1].setRawAngle(thetas.theta1);
  this->servos[FSARM_JOINT2].setRawAngle(thetas.theta2);
  this->servos[FSARM_JOINT3].setRawAngle(thetas.theta3);
  this->servos[FSARM_JOINT4].setRawAngle(thetas.theta4);
  // this->servos[FSARM_GRIPPER].setRawAngle(thetas.gripper);
}

// 设置舵机的角度 控制间隔
void  FSARM_ARM5DoF::setRawAngle(FSARM_JOINTS_STATE_T thetas,uint16_t interval){
  this->servos[FSARM_JOINT1].setRawAngleByInterval(thetas.theta1,interval,100,100,0);
  this->servos[FSARM_JOINT2].setRawAngleByInterval(thetas.theta2,interval,100,100,0);
  this->servos[FSARM_JOINT3].setRawAngleByInterval(thetas.theta3,interval,100,100,0);
  this->servos[FSARM_JOINT4].setRawAngleByInterval(thetas.theta4,interval,100,100,0);
  // this->servos[FSARM_GRIPPER].setAngle(thetas.gripper);
}

// 设置舵机的角度
void FSARM_ARM5DoF::setAngle(FSARM_JOINTS_STATE_T thetas) {
  this->servos[FSARM_JOINT1].setAngle(thetas.theta1);
  this->servos[FSARM_JOINT2].setAngle(thetas.theta2);
  this->servos[FSARM_JOINT3].setAngle(thetas.theta3);
  this->servos[FSARM_JOINT4].setAngle(thetas.theta4);
  // this->servos[FSARM_GRIPPER].setAngle(thetas.gripper);
}


// 设置单个关节的角度(关节角度)
void FSARM_ARM5DoF::setAngle(uint8_t id, float theta) {
  this->servos[id].setAngle(theta);
}

// 设置舵机的角度
void FSARM_ARM5DoF::setAngle(FSARM_JOINTS_STATE_T thetas, uint16_t interval) {
  this->servos[FSARM_JOINT1].setAngle(thetas.theta1, interval);
  this->servos[FSARM_JOINT2].setAngle(thetas.theta2, interval);
  this->servos[FSARM_JOINT3].setAngle(thetas.theta3, interval);
  this->servos[FSARM_JOINT4].setAngle(thetas.theta4, interval);
  // this->servos[FSARM_GRIPPER].setAngle(thetas.gripper, interval);
}

// 机械臂正向运动学
void FSARM_ARM5DoF::forwardKinematics(FSARM_JOINTS_STATE_T thetas,
                                      FSARM_POINT3D_T* toolPosi, float* pitch) {
  //FSARM_POINT3D_T wristPosi;  //腕关节原点的坐标
  // 求解pitch
  *pitch = thetas.theta2 + thetas.theta3 + thetas.theta4;
  // 角度转弧度
  float theta1 = radians(thetas.theta1);
  float theta2 = radians(thetas.theta2);
  float theta3 = radians(thetas.theta3);
  float theta4 = radians(thetas.theta4);
  // 计算腕关节的坐标
  toolPosi->x = cos(theta1) * (FSARM_LINK2 * cos(theta2) +
                               FSARM_LINK3 * cos(theta2 + theta3) +
                               FSARM_LINK4 * cos(theta2 + theta3 + theta4));
  toolPosi->y = sin(theta1) * (FSARM_LINK2 * cos(theta2) +
                               FSARM_LINK3 * cos(theta2 + theta3) +
                               FSARM_LINK4 * cos(theta2 + theta3 + theta4));
  toolPosi->z = -FSARM_LINK2 * sin(theta2) -
                FSARM_LINK3 * sin(theta2 + theta3) -
                FSARM_LINK4 * sin(theta2 + theta3 + theta4);
}

// 机械臂逆向运动学
FSARM_STATUS FSARM_ARM5DoF::inverseKinematics(FSARM_POINT3D_T toolPosi,
                                              float pitch,
                                              FSARM_JOINTS_STATE_T* thetas) {
  // 关节弧度
  float theta1 = 0.0;
  float theta2 = 0.0;
  float theta3 = 0.0;
  float theta4 = 0.0;
  FSARM_POINT3D_T wristPosi;  // 腕关节坐标

  // 根据工具原点距离机械臂基坐标系的直线距离判断解是否存在
  float disO2Tool =
      sqrt(pow(toolPosi.x, 2) + pow(toolPosi.y, 2) + pow(toolPosi.z, 2));
  if (disO2Tool > (FSARM_LINK2 + FSARM_LINK3 + FSARM_LINK4)) {
    return FSARM_STATUS_TOOLPOSI_TOO_FAR;
  }

  // 判断腕关节的原点是否在机械臂坐标系的Z轴上
  if (toolPosi.x == 0 && toolPosi.y == 0) {
    // 让theta1保持跟原来相同 原始角度？删除raw 应该和关节角度一样
    theta1 = radians(this->servos[FSARM_JOINT1].queryAngle());
  } else {
    // 求解theta1
    theta1 = atan2(toolPosi.y, toolPosi.x);
    thetas->theta1 = degrees(theta1);
    // 判断theta1是否合法
    if (!servos[FSARM_JOINT1].isAngleLegal(thetas->theta1)) {
      return FSARM_STATUS_JOINT1_OUTRANGE;
    }
    if (abs(servos[FSARM_JOINT1].angleReal2Raw(thetas->theta1))>135) {
      return FSARM_STATUS_RAW1_OUTRANGE;
    }
  }

  // 俯仰角, 角度转弧度
  float pitch_rad = radians(pitch);
  // 计算腕关节的位置 y轴旋转正方向为CW
  wristPosi.x = toolPosi.x - FSARM_LINK4 * cos(pitch_rad) * cos(theta1);
  wristPosi.y = toolPosi.y - FSARM_LINK4 * cos(pitch_rad) * sin(theta1);
  wristPosi.z = toolPosi.z + FSARM_LINK4 * sin(pitch_rad);
 //无穷多解的情况
  if (wristPosi.x==0&& wristPosi.y==0&&wristPosi.z==0)
  {
     return FSARM_STATUS_WRISTPOSI_TOO_NEAR;
  }
  // 计算theta3
  float b;//距离lr
  if (cos(theta1) != 0) {
    b = wristPosi.x / cos(theta1);
  } else {
    b = wristPosi.y / sin(theta1);
  }
  //三角形成立条件 Sunny add 不满足退出
  if (b> (FSARM_LINK2 + FSARM_LINK3))
  {
     return FSARM_STATUS_WRISTPOSI_TOO_FAR;
  }
  
  float cos_theta3 = (pow(wristPosi.z, 2) + pow(b, 2) - pow(FSARM_LINK2, 2) -
                      pow(FSARM_LINK3, 2)) /
                     (2 * FSARM_LINK2 * FSARM_LINK3);
  float sin_theta3 = sqrt(1 - pow(cos_theta3, 2));//省略了一个负的解
  theta3 = atan2(sin_theta3, cos_theta3);
  thetas->theta3 = degrees(theta3);
  if (!servos[FSARM_JOINT3].isAngleLegal(thetas->theta3)) {
    return FSARM_STATUS_JOINT3_OUTRANGE;
  }
      if (abs(servos[FSARM_JOINT3].angleReal2Raw(thetas->theta3))>135) {
      return FSARM_STATUS_RAW3_OUTRANGE;
    }
  // 计算theta2
  float k1 = FSARM_LINK2 + FSARM_LINK3 * cos(theta3);
  float k2 = FSARM_LINK3 * sin(theta3);
  float r = sqrt(pow(k1, 2) + pow(k2, 2));
  theta2 = atan2(-wristPosi.z / r, b / r) - atan2(k2 / r, k1 / r);
  thetas->theta2 = degrees(theta2);
  if (!servos[FSARM_JOINT2].isAngleLegal(thetas->theta2)) {
    return FSARM_STATUS_JOINT2_OUTRANGE;
  }
     if (abs(servos[FSARM_JOINT2].angleReal2Raw(thetas->theta2))>135) {
      return FSARM_STATUS_RAW2_OUTRANGE;
    }
  // 计算theta4
  theta4 = pitch_rad - (theta2 + theta3);
  thetas->theta4 = degrees(theta4);
  if (!servos[FSARM_JOINT4].isAngleLegal(thetas->theta4)) {
    return FSARM_STATUS_JOINT4_OUTRANGE;
  }
       if (abs(servos[FSARM_JOINT4].angleReal2Raw(thetas->theta4))>135) {
      return FSARM_STATUS_RAW4_OUTRANGE;
    }

  // 成功完成求解
  return FSARM_STATUS_SUCCESS;
}
// 机械臂逆向运动学 俯仰角默认为0
FSARM_STATUS FSARM_ARM5DoF::inverseKinematics(FSARM_POINT3D_T toolPosi,
                                              FSARM_JOINTS_STATE_T* thetas) {
  return inverseKinematics(toolPosi, 0.0, thetas);
}

// 机械臂末端移动, 点对点
FSARM_STATUS FSARM_ARM5DoF::move(FSARM_POINT3D_T toolPosi, float pitch) {
  FSARM_JOINTS_STATE_T thetas;
  FSARM_STATUS status =
      inverseKinematics(toolPosi, pitch, &thetas);  // 逆向运动学
  if (status == FSARM_STATUS_SUCCESS) {
    // 设置舵机的角度
    this->servos[FSARM_JOINT1].setAngle(thetas.theta1);//设置的是关节角度
    this->servos[FSARM_JOINT2].setAngle(thetas.theta2);
    this->servos[FSARM_JOINT3].setAngle(thetas.theta3);
    this->servos[FSARM_JOINT4].setAngle(thetas.theta4);
    //注: move函数并不会控制爪子
  }
  return status;
}

FSARM_STATUS FSARM_ARM5DoF::move(float tx, float ty, float tz, float pitch) {
  FSARM_POINT3D_T toolPosi;
  toolPosi.x = tx;
  toolPosi.y = ty;
  toolPosi.z = tz;
  return move(toolPosi, pitch);
}

FSARM_STATUS FSARM_ARM5DoF::move(float tx, float ty, float tz, float pitch,
                                 bool isWait) {
  FSARM_STATUS status = move(tx, ty, tz, pitch);
  if (isWait) {
    wait();
  }
  return status;
}

// home: 回归机械零点, 初始化机械臂的姿态
void FSARM_ARM5DoF::home() {
  move(FSARM_HOME_X, FSARM_HOME_Y, FSARM_HOME_Z, FSARM_HOME_PITCH, true);
}

// 返回机械臂是否空闲
bool FSARM_ARM5DoF::isIdle() {
  bool is_stop = true;
  for (int sidx = 0; sidx < FSARM_SERVO_NUM; sidx++) {
    is_stop &= this->servos[sidx].isStop();
  }
  return is_stop;
}

// 等待舵机停止
void FSARM_ARM5DoF::wait() {
  for (int sidx = 0; sidx < FSARM_SERVO_NUM; sidx++) {
    this->servos[sidx].wait();
  }
}

// 更新末端工具的坐标
void FSARM_ARM5DoF::getToolPose(FSARM_POINT3D_T* toolPosi, float* pitch) {
  FSARM_JOINTS_STATE_T thetas;
  queryAngle(&thetas);
  forwardKinematics(thetas, toolPosi, pitch);
}

// 夹爪张开
void FSARM_ARM5DoF::gripperOpen() {
  this->gripper.setAngle(GRIPPER_SERVO_ANGLE_OPEN, GRIPPER_INTERVAL_MS,
                         GRIPPER_MAX_POWER);
  delay(GRIPPER_INTERVAL_MS);
}
// 夹爪张开
void FSARM_ARM5DoF::gripperOpen(float angle) {
  this->gripper.setAngle(angle, GRIPPER_INTERVAL_MS,
                         GRIPPER_MAX_POWER);
  delay(GRIPPER_INTERVAL_MS);
}
// 夹爪闭合
void FSARM_ARM5DoF::gripperClose() {
  this->gripper.setAngle(GRIPPER_SERVO_ANGLE_CLOSE, GRIPPER_INTERVAL_MS,
                         GRIPPER_MAX_POWER);
  delay(GRIPPER_INTERVAL_MS);
}

// 夹爪闭合
void FSARM_ARM5DoF::gripperClose(float angle) {
  this->gripper.setAngle(angle, GRIPPER_INTERVAL_MS,
                         GRIPPER_MAX_POWER);
  delay(GRIPPER_INTERVAL_MS);
}
