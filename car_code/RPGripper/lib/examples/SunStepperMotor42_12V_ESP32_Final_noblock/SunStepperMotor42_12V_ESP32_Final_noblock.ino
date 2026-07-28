#include <Arduino.h>
#include <AccelStepper.h>
#include "OneButton.h"
#include <U8g2lib.h>
#include <Ticker.h> //调用Ticker.h库

// 零点开关
int ZERO_PIN = 23;                        // 零点开关引脚，遮挡输出高电平，灯灭
volatile unsigned int interruptCount = 0; //
bool zero_flag = 0;
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

int EN_PIN = 5; // 使能引脚
#define dirPin 19
#define stepPin 18
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// 按键
int MODE_BTN_PIN = 4;
int SPEED_BTN_PIN = 34;
int PAUSE_BTN_PIN = 35;
bool change_flag = 1;
int runMode = 0;
enum RunMode
{
  round1_0,
  round1_1,
  round2_0,
  round2_1
};
enum RunMode run_mode = round1_1;
int speed = 260;
int dwellTime = 4;
OneButton mode_btn(MODE_BTN_PIN, true, true); // true:按下为低电平
OneButton speed_btn(SPEED_BTN_PIN, true);
OneButton pause_btn(PAUSE_BTN_PIN, true);

void mode_click()
{
  change_flag = 1;
  Serial.println("Button mode click.");

  switch (run_mode)
  {
  case round1_0: // 初赛光电零点开关
    run_mode = round1_1;
    break;
  case round1_1: //
    run_mode = round2_0;
    break;
  case round2_0: //
    run_mode = round2_1;
    break;
  case round2_1: //
    run_mode = round1_0;
    break;
  default:
    run_mode = round1_0;
    break;
  }
}
void speed_click()
{
  change_flag = 1;
  Serial.println("Button speed click.");
  if (speed == 260)
  {
    speed = 220;
  }
  else
  {
    speed = speed + 10;
  }
}
void pause_click()
{
  change_flag = 1;
  Serial.println("Button dwell click.");
  if (dwellTime >= 5)
  {
    dwellTime = 3;
  }
  else
  {
    ++dwellTime;
  }
}
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
  mode_btn.reset(); // 清除一下按钮状态机的状态
  mode_btn.attachClick(mode_click);
  speed_btn.reset(); // 清除一下按钮状态机的状态
  speed_btn.attachClick(speed_click);
  pause_btn.reset(); // 清除一下按钮状态机的状态
  pause_btn.attachClick(pause_click);
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
  stepper.setMaxSpeed(speed);
  stepper.setAcceleration(100);

  ticker1.attach(1, callback1); // 每1秒调用callback1
  // ticker4.attach(4, callback4); // 每4秒调用callback1
  pinMode(ZERO_PIN,
          INPUT); // See http://arduino.cc/en/Tutorial/DigitalPins
  attachInterrupt(ZERO_PIN, interruptFunction, RISING);
}

void loop()
{

  if (ticker1_flag) // 每1秒执行一次
  {
    ticker1_flag = 0;
    batteryVoltage = float(analogReadMilliVolts(A0)) * 11.0 * voltage_k / 1000.0;
    // Serial.println(analogA0Volts);
    if (batteryVoltage < 10.9)
    {
      digitalWrite(Buzzer_PIN, !digitalRead(Buzzer_PIN)); // 切换电平，交替蜂鸣
      Serial.println(!digitalRead(Buzzer_PIN));
    }
    else
    {
      digitalWrite(Buzzer_PIN, LOW); // 输出LOW电平,不发声
    }
  }

  // 屏幕输出刷新,有变化执行一次
  if (change_flag)
  {
    change_flag = 0;
    Serial.println(batteryVoltage);
    u8g2.setFont(u8g2_font_unifont_t_chinese2); // use chinese2
    u8g2.firstPage();
    do
    {
      u8g2.setCursor(0, 16);
      u8g2.print("Voltage:");
      u8g2.print(batteryVoltage);
      u8g2.setCursor(0, 32);
      u8g2.print("MODE:");
      u8g2.print(run_mode);
      //u8g2.print("POSE:");
      //u8g2.print(stepper.currentPosition());
      u8g2.setCursor(0, 48);
      u8g2.print("SPEED:"); // 电池电压
      u8g2.print(speed);
      u8g2.setCursor(0, 64);
      u8g2.print("DWELL TIME:"); // 舵机电压
      u8g2.print(dwellTime);

    } while (u8g2.nextPage());
  }

  // 电机运行，电压满足条件才运行
  if (batteryVoltage > 10.9)
  {
    switch (run_mode)
    {
    case round1_0: // 初赛手动零点开关

      // Serial.println(count);
      if (count >= 3)
      {
        if (reach_flag == 0)
        {
          stepper.setSpeed(speed);
          stepper.moveTo(1066);
        }
        if (stepper.currentPosition() == 1066)
        {
          reach_flag = 1;
          ticker4.once(dwellTime, callback4);
          stepper.setCurrentPosition(0);
          change_flag = 1;
        }
        if (ticker4_flag)
        {
          ticker4_flag = 0;
          count = 0;
          reach_flag = 0;
        }
      }
      else
      {
        if (reach_flag == 0)
        {
          stepper.setSpeed(speed);
          stepper.moveTo(1067);
        }
        if (stepper.currentPosition() == 1067)
        {
          reach_flag = 1;
          ticker4.once(dwellTime, callback4);
          stepper.setCurrentPosition(0);
          change_flag = 1;
        }
        if (ticker4_flag)
        {
          ticker4_flag = 0;
          reach_flag = 0;
        }
      }

      break;
    case round1_1: // 初赛光电零点开关

      if (zero_flag == 0)
      {
        stepper.setSpeed(speed);
        stepper.runSpeed();
      }
      else
      {
        detachInterrupt(ZERO_PIN);
        stepper.setCurrentPosition(0);
        stepper.setSpeed(-100);
        while (stepper.currentPosition() != -800)
        {
          stepper.runSpeed();
        }
        delay(4000);
        zero_flag = 0;
        run_mode = round1_0;
        stepper.setCurrentPosition(0);
      }
      break;
    case round2_0: // 决赛手动零点开关
      Serial.println(round2_0);
      break;
    case round2_1: // 决赛光电零点开关
      Serial.println(round2_1);
      break;
    default:
      Serial.println("default");
      break;
    }

    stepper.run(); // 电机运行
    //mode_btn.tick();
    //speed_btn.tick();
    //dwell_btn.tick();
  }
}
