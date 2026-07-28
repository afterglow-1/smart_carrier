/*
 *
 程序名称：sunarm-set-angles
 设置机械臂关节的角度
 todo：添加手爪校准
 结构体嵌套结构体。 带oled显示
 */
#include <Arduino.h>
#include "sunnybot-arm-nvs.h"
#include <U8g2lib.h>

// 调试串口的配置
#if defined(ARDUINO_ARCH_ESP32)
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200
#endif
FSARM_ARM5DoF arm;  //机械臂对象
ARMNVS armnvs;
//oled
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21); // ESP32 Thing, HW I2C with pin remapping
 
//定义动作结构体
typedef struct {
  FSARM_JOINTS_STATE_T jointsAngle;
  bool gripperAngle;//1 open ;0 close
} SUNARM_ANGLES;

unsigned int actionsCount=0;
void setup() {
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);  // 初始化串口的波特率
  arm.init();                                 //机械臂初始化
  arm.gripperOpen();                        //测试手爪开启
  if (armnvs.readARMNVS()) {
    armnvs.printARMNVS();  //打印原有标定数据
    //使用标定数据进行标定
    arm.calibration(armnvs.nvsData.theta1, armnvs.nvsData.turn1,
                    armnvs.nvsData.theta2, armnvs.nvsData.turn2,
                    armnvs.nvsData.theta3, armnvs.nvsData.turn3,
                    armnvs.nvsData.theta4, armnvs.nvsData.turn4);
  }
  arm.setAngle(0, 5.0);  //为了体现回零确实有动作
  arm.wait();
  arm.setSpeed(150);
  arm.home();  //回参考点位置
  //屏幕初始化
  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
}

void loop() {
  // 如果提供了初始化列表，那么可以在数组定义中省略数组长度，数组长度由初始化器列表中最后一个数组元素的索引值决定
 SUNARM_ANGLES armAngles[]={
{{45.0, -130.0, 90.0, 60.0},0},
{{-90.0, -130.0, 120.0, 30.0},0},
{{-90.0, -130.0, 120.0, 45.0},0},
{{0.0, -130.0, 120.0, 35.0},1},//joint4大于35可能会触地
{{0.0, -90.0, 120.0, 30.0},0} };


  int actionsLenth = (sizeof(armAngles) / sizeof(armAngles[0]));
  //获得动作组动作数量
  DEBUG_SERIAL.print("动作组动作数量为：");
  DEBUG_SERIAL.println(actionsLenth);
  u8g2.setFont(u8g2_font_unifont_t_chinese2); // use chinese2
  u8g2.firstPage();
  do
  {
    u8g2.setCursor(0, 20);
    u8g2.print("该组含动作数:"); 
    u8g2.print(actionsLenth); 
    u8g2.setCursor(0, 40);
    u8g2.print("已做动作组数:"); 
    u8g2.print( actionsCount); 
   
  } while (u8g2.nextPage());

  for (size_t i = 0; i < actionsLenth; i++) {
    arm.setAngle(armAngles[i].jointsAngle);  // 设置舵机旋转到特定的角度
    arm.wait();               // 等待舵机旋转到目标位置
   // delay(1000);              // 等待1s
    if (armAngles[i].gripperAngle) {
      arm.gripperOpen();
    } else {
      arm.gripperClose();
    }
  }
  actionsCount++;
}