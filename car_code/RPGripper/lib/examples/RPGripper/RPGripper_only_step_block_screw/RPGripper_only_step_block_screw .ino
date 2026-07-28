//测试丝杠步进Z轴阻塞式运行，确认Z轴移动部件处在中间位置，上下运动各一圈，一圈8mm，上电位置为初始零位。
//runToNewPosition – 电机运行到用户指定位置值，目标位置为绝对位置。此函数将“block”程序运行。即电机没有到达目标位置前，Arduino将不会继续执行后续程序内容。
#include <Arduino.h>
#include <AccelStepper.h>//https://github.com/waspinator/AccelStepper
//http://www.airspayce.com/mikem/arduino/AccelStepper/
//https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
#include "OneButton.h"
#include <U8g2lib.h>
#include <Ticker.h> //调用Ticker.h库
//GT2 齿距2mm  同步轮 20齿 距离换算 40mm/圈
//T8丝杠 导程8mm 每圈8mm
//步距角 1.8  200个 细分数 默认16细分 200*16=3200每圈
// 是否使用零点开关
//http://www.taichi-maker.com/homepage/reference-index/arduino-library-index/accelstepper-library/

//直线导轨上止点（TDC）和下止点（BDC）
#define TDC -1600 //向上20MM
#define BDC 1600 //向下20MM
#define USE_ORIGIN_SWITCH

#ifdef USE_ORIGIN_SWITCH
int ORIGIN_PIN = 26;                      // 零点开关引脚，遮挡输出高电平，灯灭
volatile unsigned int interruptCount = 0; //
bool zero_flag = 0;
#endif
// 移动距离转换为步进电机脉冲数
long mm2step(long postion) {
  return postion * 400;
}
bool reach_flag = 0;
// 外部中断回调函数1
void interruptFunction()
{

  zero_flag = 1;

  // interruptCount++;
}
// 定时器
Ticker ticker1; // 声明Ticker对象
Ticker ticker4; // 声明Ticker对象
bool ticker1_flag = 0;
void callback1() // 回调函数1
{
  ticker1_flag = 1;
}

bool ticker4_flag = 0;
int count = 1;
void callback4() // 回调函数1
{
  ticker4_flag = 1;
  ++count;
}

// 蜂鸣器
int Buzzer_PIN = 4; // 管脚4连接到蜂鸣器元件的基极

// 使用闭环步进

// 200脉冲/圈 16细分 

// 使用步进引脚

int EN_PIN = 32; // 使能引脚
#define dirPin 25
#define stepPin 33
#define motorInterfaceType 1 //< Stepper Driver, 2 driver pins required

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);


// oled
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21); // ESP32 Thing, HW I2C with pin remapping
float voltage_k = 0.99;
float batteryVoltage = 0;
void setup()
{

  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function

  Serial.begin(115200);
  Serial.println("test");

  pinMode(Buzzer_PIN, OUTPUT);   // 设置pinBuzzer脚为输出状态
  digitalWrite(Buzzer_PIN, LOW); // 测试蜂鸣器
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // 使能步进电机 低电平有效
  digitalWrite(Buzzer_PIN, HIGH); // 测试蜂鸣器
  delay(100);
  digitalWrite(Buzzer_PIN, LOW); // 测试蜂鸣器
                                 // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(60000);//最多转速1200rpm 每秒20转 *3200脉冲=64000脉冲/秒 最大不要超过60000
  stepper.setAcceleration(20000);

  ticker1.attach(1, callback1); // 每1秒调用callback1
  // ticker4.attach(4, callback4); // 每4秒调用callback1
  pinMode(ORIGIN_PIN,
          INPUT); // See http://arduino.cc/en/Tutorial/DigitalPins
  attachInterrupt(ORIGIN_PIN, interruptFunction, RISING);
}

void loop()
{

  stepper.runToNewPosition(-3200);//3200一圈  正负号控制方向检查以下Z向实际运动方向 是上还是下

   stepper.runToNewPosition(0);
   stepper.runToNewPosition(3200);//3200一圈  正负号控制方向检查以下Z向实际运动方向 是上还是下
   stepper.runToNewPosition(0);



}
