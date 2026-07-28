#include <Arduino.h>
#include <AccelStepper.h>//https://github.com/waspinator/AccelStepper
//http://www.airspayce.com/mikem/arduino/AccelStepper/
//https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
#include "OneButton.h"
#include <U8g2lib.h>
#include <Ticker.h> //调用Ticker.h库
//GT2 齿距2mm  同步轮 20齿 距离换算 40mm/圈
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
  return postion * 80;
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
int Buzzer_PIN = 2; // 管脚2连接到蜂鸣器元件的基极

// 使用闭环步进

// 200脉冲/圈 16细分 3200个 三等分 1,066.666 1067 1067 1066
// 每秒220个脉冲 14.5秒

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
// 使用M4轴步进引脚

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
  // pinMode(MODE_BTN_PIN, INPUT);       // 初始化模式按键引脚为输入
  // pinMode(SPEED_BTN_PIN, INPUT);      // 初始化速度按键引脚为输入
  // pinMode(DWELL_TIME_BTN_PIN, INPUT); // 初始化停顿时间按键引脚为输入
  digitalWrite(EN_PIN, LOW);      // 使能步进电机 低电平有效
  digitalWrite(Buzzer_PIN, HIGH); // 测试蜂鸣器
  delay(100);
  digitalWrite(Buzzer_PIN, LOW); // 测试蜂鸣器
                                 // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(800);
  stepper.setAcceleration(500);

  ticker1.attach(1, callback1); // 每1秒调用callback1
  // ticker4.attach(4, callback4); // 每4秒调用callback1
  pinMode(ORIGIN_PIN,
          INPUT); // See http://arduino.cc/en/Tutorial/DigitalPins
  attachInterrupt(ORIGIN_PIN, interruptFunction, RISING);
}

void loop()
{
if ( stepper.currentPosition() == 0 ){
    // 电机转动半周
    stepper.moveTo(BDC);   //  设定需要移动的绝对位置         
  
    
  } else if ( stepper.currentPosition() == 1600 ){
    // 电机转动半周
    stepper.moveTo(TDC);            

  }         
  
    stepper.run(); // 电机运行

}
