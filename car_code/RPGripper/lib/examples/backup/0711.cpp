/*
 * @Author: distroyer of the world 3210101752@zju.edu.cn
 * @Date: 2023-11-30 00:26:23
 * @LastEditors: igcxl acer5502@gmail.com
 * @LastEditTime: 2024-07-10 22:42:56
 * @FilePath: \New_Gripper_git\src\main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// 一路IIC0做主机用于屏幕显示，另一路IIC1做主机用于和移动机器人主控通讯
// http://www.taichi-maker.com/homepage/reference-index/arduino-library-index/wire-library/
// 主机给从机发送指令
/*
指令格式P+n,  P2P运动到Pn点。
指令格式X+n,X方向运动n毫米。
指令格式Y+n,Y方向运动n毫米。
指令格式Z+n,Z方向运动n毫米。
指令格式F+n,获取移动机器人状态信息。
移动机器人状态码：
0x00  待启动
0x11 走目标点n状态，走完后跳到0x66状态
0x22 修改轮式里程计X值,修改后跳到0x11状态
0x33 修改轮式里程计Y值,修改后跳到0x11状态
0x44 修改轮式里程计Z值,修改后跳到0x11状态
0x66  机器人一键启动后准备就绪

*/
#include <Arduino.h>
#include "FashionStar_UartServoProtocol.h"  // 串口总线舵机通信协议
#include "FashionStar_UartServo.h"          // Fashion Star串口总线舵机
#include "FashionStar_SmartGripper.h"       //自适应夹爪
#include <AccelStepper.h>
#include "OneButton.h"
#include <U8x8lib.h>
#include <Ticker.h> 
#include <Wire.h>

// 串口总线舵机配置
#define ARM_BASE_SERVO_ID 0 // 舵机0的ID号 基座舵机
#define GRIPPER_SERVO_ID 4 // 舵机4的ID号 手爪
#define STORAGE_SERVO_ID 5 // 舵机5的ID号 载物盘舵机
#define SERVO_BAUDRATE 115200 // 波特率
#define USE_ARM_A  //使用主力机械臂A

/**主力机械臂A舵机参数****************************************/
// 爪子的配置
#ifdef USE_ARM_A
#define GRIPPER_OPEN_ANGLE 15.0 // 爪子张开时的角度
#define GRIPPER_CLOSE_ANGLE -45.0 // 爪子闭合时的角度
#define GRIPPER_OPE_MAX_ANGLE 30  //爪子张开最大大角度

#define ARM_BASE_SERVO_HOME_ANGLE   -90  //机械臂发车初始角度
#define ARM_BASE_SERVO_OUT_ANGLE   0  //机械臂底部舵机朝车外角度
#define RM_BASE_SERVO_IN_ANGLE    -125  //机械臂底部舵机朝车内的角度

#endif

/**替补机械臂B舵机参数****************************************/
#ifndef USE_ARM_A
#define GRIPPER_OPEN_ANGLE 15.0 // 爪子张开时的角度
#define GRIPPER_CLOSE_ANGLE -45.0 // 爪子闭合时的角度
#define GRIPPER_OPE_MAX_ANGLE 30  //爪子张开最大大角度

#define ARM_BASE_SERVO_HOME_ANGLE   -90  //机械臂发车初始角度
#define ARM_BASE_SERVO_OUT_ANGLE   0  //机械臂底部舵机朝车外角度
#define RM_BASE_SERVO_IN_ANGLE    -125  //机械臂底部舵机朝车内的角度

#endif
//        0出发位置   1R  2G  3B  储物盘三个盘位正对机械臂的角度
int storage[4]={0,91,-1,-89};  

/**舵机***************************************************/
// 创建舵机的通信协议对象
FSUS_Protocol protocol(SERVO_BAUDRATE);
// 创建舵机的实例
FSUS_Servo armBaseServo(ARM_BASE_SERVO_ID, &protocol); // 机械臂舵机

FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol); // 载物盘舵机

FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol); // 手爪

// 创建智能机械爪实例
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE,GRIPPER_CLOSE_ANGLE);


void arm_go_home();
void  arm_out();
void  arm_in();
void setup()
{
    /**舵机******************************************************************/
  // DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 软串口
  protocol.init();    // 舵机通信协议初始化  
 armBaseServo.init();      // 机械臂旋转基座舵机初始化
  storageServo.init();      // 储物盘舵机初始化
  gripper.init();      // 手爪舵机初始化

  // 参数配置
    gripper.setMaxPower(400); // 设置最大功率，单位mW
 armBaseServo.setSpeed(500);      // 舵机0初始化 机械臂舵机
  storageServo.setSpeed(500);      // 舵机1初始化 储物盘
arm_go_home();
delay(100);

}


void loop()
{
arm_out();
}


void arm_go_home(){

  //stepper.runToNewPosition();
  gripper.close();
  armBaseServo.setAngle(ARM_BASE_SERVO_HOME_ANGLE);
  storageServo.setAngle(storage[0]);
  armBaseServo.wait();
  storageServo.wait();
}

void arm_out()
{

  //stepper.runToNewPosition();
  //gripper.close();
  armBaseServo.setAngle(ARM_BASE_SERVO_OUT_ANGLE);  
  armBaseServo.wait();
 
}

void arm_in()
{

  //stepper.runToNewPosition();
  //gripper.close();
  armBaseServo.setAngle(RM_BASE_SERVO_IN_ANGLE);  
  armBaseServo.wait();
 
}