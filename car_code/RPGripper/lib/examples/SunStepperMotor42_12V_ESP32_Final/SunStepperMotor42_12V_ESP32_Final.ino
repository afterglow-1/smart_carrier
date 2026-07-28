#include <Arduino.h>
#include <AccelStepper.h>
#include "OneButton.h"
#include <U8g2lib.h>
#include <Ticker.h> //调用Ticker.h库
#include <Preferences.h>
#define RW_MODE false
#define RO_MODE true
// todo 第二轮位置6 ；开机步进电机使能控制？
//  最大速度
const int MAXSpeed = 500;         // 步进电机最大速度
const int acceleration = 200;     // 步进电机最大加速度,初赛加速度

int speed = 400;                  // 步进电机设定速度
int& accelerationH = speed; // 步进电机最大加速度,决赛提高加速度
int accelerationHigh=400; //步进电机最大加速度,决赛提高加速度
unsigned long last_time = 0;
unsigned long used_time = 0;
// 阻塞式步进运行，在暂停时更新屏幕等
enum RunMode
{
  round1_0, // 初赛手动回零模式
  round1_1, // 初赛自动回零模式
  round2_0, // 决赛手动回零模式
  round2_1  // 决赛自动回零模式
};
enum RunMode run_mode = round1_1;
//  零点开关
int ZERO_PIN = 23;                        // 零点开关引脚，遮挡输出高电平，灯灭
volatile unsigned int interruptCount = 0; //
bool zero_flag = 0;
bool isZero = 0;
bool reach_flag = 0;
bool error_flag = 0;//寻找接近开关错误标记位
bool change_flag = 1;
bool isInterruptActive = 0;
// 外部中断回调函数1
void interruptFunction()
{

  zero_flag = 1;

  // interruptCount++;
}
// 定时器
Ticker ticker1; // 声明Ticker对象

void callback1() // 回调函数1
{
  change_flag = 1;
}
Ticker ticker4; // 声明Ticker对象
// bool ticker1_flag = 0;

bool ticker4_flag = 1;
int count = 0;          // 计数次数，取值范围1-3
int count6 = 0;         // stop 6 times count,取值范围1-6
void callback4()        // 回调函数1
{
  ticker4_flag = 1;
  change_flag = 1;
  ++count;
  ++count6;
  if(last_time!=0){
  used_time = millis() - last_time;
  }
  if (count >= 4)
  {
    count = 1;
  }
  if (count6 >= 7)
  {
    count6 = 1;
  }

  if ((run_mode == round1_0) || (run_mode == round1_1))
  {
    if (count == 1)
    {
      last_time = millis();
    }
  }
  else if ((run_mode == round2_0) || (run_mode == round2_1))
  {
    if (count6 == 1)
    {
      last_time = millis();
    }
  }
}

// 蜂鸣器
int Buzzer_PIN = 2; // 管脚2连接到蜂鸣器元件的基极

// 使用闭环步进

// 200脉冲/圈 16细分 3200个 三等分 1,066.666 1067 1067 1066
// 每秒220个脉冲 14.5秒

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
// 使用M4轴步进引脚

const int EN_PIN = 5; // 使能引脚
#define dirPin 19
#define stepPin 18
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// 按键
const int START_BTN_PIN = 0;
const int MODE_BTN_PIN = 4;
const int SPEED_BTN_PIN = 34;
const int PAUSE_BTN_PIN = 35;
bool start_flag = 0;
bool save_flag = 0;


int dwellTime = 4;

typedef struct
{
  uint64_t nvs_chipid;
  int nvs_runMode;
  int nvs_speed;
  int nvs_dwellTime;

} NVS_DATA;

NVS_DATA nvsData;
Preferences prefs;

// 读取NVS数据
bool readNVS()
{
  // 只读模式打开或创建RTNVS命名空间
  prefs.begin("RTNVS", RO_MODE);
  bool RTInit = prefs.isKey(
      "chipid"); // Test for the existence of the "already initialized" key.
  if (RTInit == false)
  {
    prefs.end();
    return false;
  }
  else
  {
    nvsData.nvs_chipid = prefs.getULong64("chipid", 0);
    nvsData.nvs_runMode = prefs.getInt("mode", 1);
    nvsData.nvs_speed = prefs.getInt("speed", 300);
    nvsData.nvs_dwellTime = prefs.getInt("time", 4);
    prefs.end();
    return true;
  }
}

// 保存数据到NVS
void writeNVS()
{
  prefs.begin("RTNVS", RW_MODE);
  // The chip ID is essentially its MAC address(length: 6 bytes).
  if (nvsData.nvs_chipid == 0)
  {
    prefs.putULong64("chipid", ESP.getEfuseMac());
    nvsData.nvs_chipid = ESP.getEfuseMac();
  }

  prefs.putInt("mode", run_mode);
  prefs.putInt("speed", speed);
  prefs.putInt("time", dwellTime);
  prefs.end(); // 关闭当前命名空间
}

OneButton start_btn(START_BTN_PIN, true, true); // true:按下为低电平
OneButton mode_btn(MODE_BTN_PIN, true, true);   // true:按下为低电平
OneButton speed_btn(SPEED_BTN_PIN, true);
OneButton pause_btn(PAUSE_BTN_PIN, true);

void start_click()
{
  start_flag = 1;
  // save_flag=1;
}

void mode_click()
{
  change_flag = 1;
  save_flag = 1;
  // Serial.println("Button mode click.");
  //  初始化状态量
  count = 0;
  isZero == 0;
  stepper.setCurrentPosition(0);
  switch (run_mode)
  {
  case round1_0:
    run_mode = round1_1; // 切换为初赛光电零点开关
    dwellTime = 4;       // 暂停时间固定为4
    // stepper.setAcceleration(acceleration); // 设置电机加速度低
    // interrupts();
    break;
  case round1_1: //
    run_mode = round2_0;
    //speed = 400;
    // stepper.setAcceleration(accelerationHigh); // 设置电机高加速度
    break;
  case round2_0: //
    run_mode = round2_1;
    //speed = 400;//500
    // stepper.setAcceleration(accelerationHigh); // 设置电机高加速度
    // interrupts();
    break;
  case round2_1: //
    run_mode = round1_0;
    dwellTime = 4; // 暂停时间固定为4
    // stepper.setAcceleration(acceleration); // 设置电机加速度低
    break;
  default:
    run_mode = round1_0; // 切换为初赛光电零点开关
    dwellTime = 4;       // 暂停时间固定为4
    // stepper.setAcceleration(acceleration); // 设置电机加速度低
    break;
  }
}
void speed_click()
{
  change_flag = 1;
  save_flag = 1;
  Serial.println("Button speed click.");
  if (speed == MAXSpeed)
  {
    speed = 300;
  }
  else
  {
    speed = speed + 100;
  }
  stepper.setMaxSpeed(speed);
}
void pause_click()
{
  change_flag = 1;
  save_flag = 1;
  // Serial.println("Button PAUSE click.");
  if ((run_mode == round2_0) || (run_mode == round2_1))
  {

    if (dwellTime >= 5)
    {
      dwellTime = 3;
    }
    else
    {
      ++dwellTime;
    }
  }
  else
  {
    dwellTime = 4;
  }
}

// 回零点
void goHome()
{

  // 开启外部中断
  if (isInterruptActive == 0)
  {
    attachInterrupt(digitalPinToInterrupt(ZERO_PIN), interruptFunction, RISING);
    isInterruptActive = 1;
  }
  // 寻找零点开关
  if (zero_flag == 0)
  {
    if (stepper.currentPosition() <= 3300)
    {
      stepper.setSpeed(300);
      stepper.run();
    }
    else
    {

      error_flag = 1;
      change_flag = 1;
      Serial.println("go home error");
    }
  }
  else
  { // 关闭外部中断
    if (isInterruptActive == 1)
    {
      detachInterrupt(digitalPinToInterrupt(ZERO_PIN));
      isInterruptActive = 0;
    }
    error_flag = 0;
    stepper.setCurrentPosition(0);
    stepper.stop();
    stepper.runToPosition();
    stepper.setSpeed(-200);
    while (stepper.currentPosition() != -800) // 反转90度
    {
      stepper.run();
    }
    zero_flag = 0;
    isZero = 1;
    stepper.setCurrentPosition(0);
    ticker4_flag = 0;
    ticker4.once(dwellTime, callback4);
  }
}
// 每圈停六次
void pause_6()
{
  if (ticker4_flag) // 每定时一次执行
  {
    ticker4_flag = 0;
    Serial.println(count6);
    if (count == 3)
    {
      stepper.setCurrentPosition(0);
      stepper.setMaxSpeed(speed);
      stepper.runToNewPosition(534);
      stepper.setSpeed(speed);
      ticker4.once(dwellTime, callback4);
    }
    else
    {
      stepper.setCurrentPosition(0);
      stepper.setMaxSpeed(speed);
      stepper.setSpeed(speed);
      stepper.runToNewPosition(533);
      ticker4.once(dwellTime, callback4);
    }
  }
}
// 每圈停3次
void pause_3()
{
  if (ticker4_flag) // 每定时一次执行
  {
    ticker4_flag = 0;
    Serial.println(count);
    if (count == 3)
    {
      stepper.setCurrentPosition(0);
      stepper.setMaxSpeed(speed);
       stepper.setSpeed(speed);
      stepper.runToNewPosition(1066);
      /*
      while (stepper.currentPosition() != 1066)
      {
        stepper.moveTo(1066);
        stepper.run();
      }
      */
      ticker4.once(dwellTime, callback4);
    }
    else
    {
      stepper.setCurrentPosition(0);
      stepper.setMaxSpeed(speed);
      stepper.setSpeed(speed);
      stepper.runToNewPosition(1067);
      /*
      while (stepper.currentPosition() != 1067)
      {
        stepper.moveTo(1066);
        stepper.run();
      }
      */

      ticker4.once(dwellTime, callback4);
    }
  }
}
// oled
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21); // ESP32 Thing, HW I2C with pin remapping
float voltage_k = 0.99;
float batteryVoltage = 0;

void setup()
{

  if (readNVS())
  {
    // 若有，从NVS中加载值
    run_mode = RunMode(nvsData.nvs_runMode); // 强制类型转换为枚举类型
    speed = nvsData.nvs_speed;
    dwellTime = nvsData.nvs_dwellTime;
  }
  else
  { // 若无，将现有值写入NVS
    writeNVS();
  }
  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function

  Serial.begin(115200);
  batteryVoltage = float(analogReadMilliVolts(A0)) * 11.0 * voltage_k / 1000.0;
  Serial.print("Voltage:");
  Serial.println(batteryVoltage);
  mode_btn.reset(); // 清除一下按钮状态机的状态
  mode_btn.attachClick(mode_click);
  speed_btn.reset(); // 清除一下按钮状态机的状态
  speed_btn.attachClick(speed_click);
  pause_btn.reset(); // 清除一下按钮状态机的状态
  pause_btn.attachClick(pause_click);
  pinMode(Buzzer_PIN, OUTPUT);   // 设置pinBuzzer脚为输出状态
  digitalWrite(Buzzer_PIN, LOW); // 测试蜂鸣器
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);      // 使能步进电机 低电平有效
  digitalWrite(Buzzer_PIN, HIGH); // 测试蜂鸣器
  delay(100);
  digitalWrite(Buzzer_PIN, LOW);         // 测试蜂鸣器
                                         // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(MAXSpeed);         // 电机最大速度
  stepper.setAcceleration(acceleration); // 设置电机加速度

  ticker1.attach(1, callback1); // 每1秒调用callback1
  //  ticker4.attach(4, callback4); // 每4秒调用callback1
  pinMode(ZERO_PIN,
          INPUT); // See http://arduino.cc/en/Tutorial/DigitalPins
  attachInterrupt(digitalPinToInterrupt(ZERO_PIN), interruptFunction, RISING);
  isInterruptActive = 1;
  start_btn.reset(); // 清除一下按钮状态机的状态
  start_btn.attachClick(start_click);
}

void loop()
{
  start_btn.tick();
  mode_btn.tick();
  speed_btn.tick();
  pause_btn.tick();
  // 按下START键后保存数据到NVS
  if (save_flag)
  {
    save_flag = 0;
    writeNVS();
  }
  // 屏幕输出刷新,有变化执行一次
  if (change_flag)
  {
    change_flag = 0;
    batteryVoltage = float(analogReadMilliVolts(A0)) * 11.0 * voltage_k / 1000.0;
    if (batteryVoltage < 10.9)
    {
      digitalWrite(Buzzer_PIN, !digitalRead(Buzzer_PIN)); // 切换电平，交替蜂鸣
      Serial.println(!digitalRead(Buzzer_PIN));
    }
    else
    {
      digitalWrite(Buzzer_PIN, LOW); // 输出LOW电平,不发声
    }
    Serial.println(batteryVoltage);
    u8g2.setFont(u8g2_font_unifont_t_chinese2); // use chinese2
    u8g2.firstPage();
    do
    {
      u8g2.setCursor(0, 16);
      u8g2.print("Voltage:");
      u8g2.print(batteryVoltage);
      u8g2.print("V");
     
      u8g2.setCursor(0, 32);
      u8g2.print("MODE:");
      u8g2.print(run_mode);
      u8g2.print(" POSE:");
      if ((run_mode == round1_0) || (run_mode == round1_1))
      {
        u8g2.print(count);
      }
      else if ((run_mode == round2_0) || (run_mode == round2_1))
      {
        u8g2.print(count6);
      }

      u8g2.setCursor(0, 48);
      u8g2.print("PAUSE:"); // 舵机电压
      u8g2.print(dwellTime);
      u8g2.print("s ");      // 舵机电压
      u8g2.print(used_time); // 舵机电压
      u8g2.print("ms");      // 舵机电压

      u8g2.setCursor(0, 64);

      u8g2.print("SPEED:"); // 电池电压
      u8g2.print(speed);
       if (error_flag)
      {
        u8g2.print(" ERR");
      }
    } while (u8g2.nextPage());
  }

  // 电机运行，按下启动按钮才运行
  if (start_flag)
  {
    if (ticker1.active())
    {
      ticker1.detach();
    }
    switch (run_mode)
    {
    case round1_0:                           // 初赛手动零点开关
      stepper.setAcceleration(acceleration); // 设置电机加速度低
      if (isZero == 0)                       // 如果没有回零
      {
        stepper.setCurrentPosition(0);
        isZero = 1;
        ticker4_flag = 0;
        ticker4.once(dwellTime, callback4);
      }
      else
      {
        pause_3();
      }

      break;
    case round1_1:                           // 初赛光电零点开关
      stepper.setAcceleration(acceleration); // 设置电机加速度低
      if (isZero == 0)                       // 如果没有回零
      {
        goHome();
      }
      else
      {
        pause_3();
      }

      break;
    case round2_0:                               // 决赛手动零点开关
    if (speed!=300)
    {
      accelerationHigh=accelerationH-40; //步进电机最大加速度,决赛提高加速度
    }
    
   
      stepper.setAcceleration(accelerationHigh); // 设置电机高加速度
      if (isZero == 0)                           // 如果没有回零
      {
        stepper.setCurrentPosition(0);
        isZero = 1;
        ticker4_flag = 0;
        ticker4.once(dwellTime, callback4);
      }
      else
      {

        pause_6();
      }

      break;
    case round2_1:                               // 决赛光电零点开关
      if (speed!=300)
    {
      accelerationHigh=accelerationH-40; //步进电机最大加速度,决赛提高加速度
    }
    
      stepper.setAcceleration(accelerationHigh); // 设置电机高加速度
      if (isZero == false)                       // 如果没有回零
      {
        goHome();
      }
      else
      {

        pause_6();
      }
      break;
    default:
      Serial.println("default");
      break;
    }
  }
  else
  {

    if (ticker1.active() == false)
    {
      ticker1.attach(1, callback1); // 每1秒调用callback1
    }
  }
}
