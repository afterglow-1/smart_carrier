/*
 *
 程序名称：sunarm-set-angles
 设置机械臂关节的角度
 带电压检测
 todo：添加手爪校准
 结构体嵌套结构体。 带oled显示
 */
#include <Arduino.h>
#include "sunnybot-arm-nvs.h"
#include <U8g2lib.h>
#include <ESPAsyncWebServer.h>
#define ButtonPin 0
// 调试串口的配置
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200

FSARM_ARM5DoF arm; //机械臂对象
ARMNVS armnvs;     // nvs对象
// oled
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21); // ESP32 Thing, HW I2C with pin remapping
float voltage_k = 0.99;                                                                                   //电压修正系数
//定义动作结构体
typedef struct
{
  FSARM_JOINTS_STATE_T jointsAngle;
  bool gripperAngle; // 1 open ;0 close
} SUNARM_ANGLES;
// 如果提供了初始化列表，那么可以在数组定义中省略数组长度，数组长度由初始化器列表中最后一个数组元素的索引值决定
//低手爪会碰撞，修改关节3
SUNARM_ANGLES armAngles0[] = {
    {{45.0, -130.0, 90.0, 60.0}, 0},
    {{-90.0, -130.0, 100.0, 30.0}, 0},
    {{-90.0, -130.0, 100.0, 35.0}, 0},
    {{0.0, -130.0, 100.0, 30.0}, 1}, // joint4大于35可能会触地
    {{0.0, -90.0, 100.0, 30.0}, 0}};

    SUNARM_ANGLES armAngles1[] = {
    {{45.0, -130.0, 90.0, 60.0}, 0},
    {{-90.0, -130.0, 100.0, 30.0}, 1},
    {{-90.0, -130.0, 100.0, 35.0}, 0},
    {{0.0, -130.0, 100.0, 0.0}, 1}, // joint4大于35可能会触地
    {{0.0, -90.0, 100.0, 30.0}, 0}};

unsigned int actionsCount = 0;
// Timer variables
unsigned long lastTime = 0;
void setup()
{
  pinMode(ButtonPin, INPUT_PULLUP);
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化串口的波特率
  // set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
  analogSetPinAttenuation(A0, ADC_2_5db); //可测量的输入电压范围100 mV ~ 1250 mV
  analogSetPinAttenuation(A3, ADC_2_5db); //可测量的输入电压范围100 mV ~ 1250 mV
                                          //屏幕初始化
  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function

  arm.init(); //机械臂初始化

  if (armnvs.readARMNVS())
  {

    //使用标定数据进行标定
    arm.calibration(armnvs.nvsData.theta1, armnvs.nvsData.turn1,
                    armnvs.nvsData.theta2, armnvs.nvsData.turn2,
                    armnvs.nvsData.theta3, armnvs.nvsData.turn3,
                    armnvs.nvsData.theta4, armnvs.nvsData.turn4);
    arm.gripper.angleGripperOpen = armnvs.nvsData.gripperOpen;
    arm.gripper.angleGripperClose = armnvs.nvsData.gripperOpen - 85.0;
    armnvs.printARMNVS(); //打印原有标定数据
  }
  arm.gripperOpen(); //测试手爪开启
  uint32_t countC = 0;
  while (digitalRead(ButtonPin) == HIGH)
  {
    if ((millis() - lastTime) > 2000)
    {
      countC++;
      DEBUG_SERIAL.print(
          "按下按钮IO0,IO0状态:");
      DEBUG_SERIAL.print(!digitalRead(ButtonPin));
      DEBUG_SERIAL.print(";累计次数:");
      DEBUG_SERIAL.println(countC);
      lastTime = millis();
    }
  }
  DEBUG_SERIAL.println("已按下按钮IO0。");

  arm.setAngle(0, 5.0); //为了体现回零确实有动作
  arm.wait();
  arm.setSpeed(150); //设置舵机速度
  arm.home();        //回参考点位置
}
void loop()
{

  float analogA0Volts = float(analogReadMilliVolts(A0)) * 11.0 * voltage_k / 1000.0;
  float analogA3Volts = float(analogReadMilliVolts(A3)) * 11.0 * voltage_k / 1000.0;
  if (analogA0Volts <= 10.9)
  {
    DEBUG_SERIAL.print("电池电压低，请马上充电！");
  }
  else
  {
    DEBUG_SERIAL.print("电池电压正常。");
  }
  int actionsLenth = (sizeof(armAngles0) / sizeof(armAngles0[0]));
  //获得动作组动作数量
  DEBUG_SERIAL.print("动作组动作数量为：");
  DEBUG_SERIAL.println(actionsLenth);
  //屏幕输出刷新
  u8g2.setFont(u8g2_font_unifont_t_chinese2); // use chinese2
  u8g2.firstPage();
  do
  {
    u8g2.setCursor(0, 20);
    u8g2.print("该组含动作数:");
    u8g2.print(actionsLenth);
    u8g2.setCursor(0, 40);
    u8g2.print("已做动作组数:");
    u8g2.print(actionsCount);
    u8g2.setCursor(0, 60);
    u8g2.print("BV:"); //电池电压
    u8g2.print(analogA0Volts);
    u8g2.print("SV:"); //舵机电压
    u8g2.print(analogA3Volts);

  } while (u8g2.nextPage());
//执行动作组0
  for (size_t i = 0; i < actionsLenth; i++)
  {
    arm.setAngle(armAngles0[i].jointsAngle); // 设置舵机旋转到特定的角度
    arm.wait();                             // 等待舵机旋转到目标位置
                                            // delay(1000);              // 等待1s
    if (armAngles0[i].gripperAngle)
    {
      arm.gripperOpen();
    }
    else
    {
      arm.gripperClose();
    }
  }

  //执行动作组1
  for (size_t i = 0; i < actionsLenth; i++)
  {
    arm.setAngle(armAngles1[i].jointsAngle); // 设置舵机旋转到特定的角度
    arm.wait();                             // 等待舵机旋转到目标位置
                                            // delay(1000);              // 等待1s
    if (armAngles1[i].gripperAngle)
    {
      arm.gripperOpen();
    }
    else
    {
      arm.gripperClose();
    }
  }
  actionsCount++;
}

