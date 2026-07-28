/*
 * 设置机械臂关节的角度
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */
#include "FashionStar_Arm5DoF.h"

FSARM_ARM5DoF arm; //机械臂对象

void setup(){
    arm.init(); //机械臂初始化
}

void loop(){
    // 如果提供了初始化列表，那么可以在数组定义中省略数组长度，数组长度由初始化器列表中最后一个数组元素的索引值决定
    FSARM_JOINTS_STATE_T thetas[]=
    {{45.0,-130.0, 90.0,60.0},
    {-90.0,-130.0, 120.0,30.0}

    };

    //

    // 设置
    arm.setAngle(thetas[0]);  // 设置舵机旋转到特定的角度
    arm.wait();            // 等待舵机旋转到目标位置
    
    delay(1000); // 等待1s

;
    
    arm.setAngle(thetas[1]);  // 设置舵机旋转到特定的角度
    arm.wait();            // 等待舵机旋转到目标位置
    delay(1000);
}