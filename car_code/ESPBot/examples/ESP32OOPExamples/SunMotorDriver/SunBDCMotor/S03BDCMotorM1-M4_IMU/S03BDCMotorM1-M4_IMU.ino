/*
 * This example uses the SunBDCMotor library to drive each motor with the
 *  AS4950 4 channel Motor Driver Board for Arduino forward, then backward.
 * 每个轮子同时正转后反转，同时正转亮红灯。反转亮蓝灯
 * 请检查轮子编号和理论建模编号是否一致，修改map2model
 * 请检查转向是否正确，修改flipMotors
 */

#include "OOPConfig.h"

BDCMotor motors;  //建立直流电机实例对象
float lastyaw = 0;

#ifdef USE_FASTLED
CRGB leds[NUM_LEDS];  // 建立光带leds实例对象
void ledShow(int bright) {
  if (bright > 0) {
    leds[0] = CRGB::Red;
    FastLED.setBrightness(bright);
    FastLED.show();
  } else {
    bright = -bright;
    leds[0] = CRGB::Blue;
    FastLED.setBrightness(bright);
    FastLED.show();
  }
}
#endif

void setup() {
  motors.init();
  motors.map2model(1, 2, 3,
                   4);                                                            //映射实际接线电机编号到理论建模编号，默认顺序1，2，3，4
  motors.flipMotors(FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2], FLIP_MOTOR[3]);  //根据实际转向进行调整false or true   绿色PCB电机
  // motors.flipMotors(false, false, false,false);
  // //根据实际转向进行调整false or true 黑色PCB电机
#ifdef USE_FASTLED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
#endif
  Serial.begin(115200);
  Serial.println("AS4950 Quad Motor Driver Board for Arduino");
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // 设置串口1波特率为115200  用做IM948通信口
  delay(3000);                                    //等待
                                                  // 唤醒传感器，并配置好传感器工作参数，然后开启主动上报---------------
  Cmd_03();                                       // 2 唤醒传感器
  /**
       * 设置设备参数
     * @param accStill    惯导-静止状态加速度阀值 单位dm/s?
     * @param stillToZero 惯导-静止归零速度(单位cm/s) 0:不归零 255:立即归零
     * @param moveToZero  惯导-动态归零速度(单位cm/s) 0:不归零
     * @param isCompassOn 1=需开启磁场 0=需关闭磁场
     * @param barometerFilter 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
     * @param reportHz 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
     * @param gyroFilter    陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
     * @param accFilter     加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
     * @param compassFilter 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
     * @param Cmd_ReportTag 功能订阅标识
     */
  Cmd_12(5, 255, 0, 1, 3, 2, 2, 4, 9, 0x00E0);  // 7 设置设备参数(内容1)
  Cmd_19();
  motors.setSpeeds(100, 100, 100, 100);
  Serial.println("速度PWM:100");
#ifdef USE_FASTLED
  ledShow(100);
#endif
  delay(5000);
  motors.motorsBrake();  //观察急刹效果
  Serial.println("急刹");
  delay(1000);
  motors.setSpeeds(-100, -100, -100, -100);
  Serial.println("速度PWM:-100");
#ifdef USE_FASTLED
  ledShow(-100);
#endif
  delay(5000);
}

void loop() {

  if (lastyaw != rpy[2]) {
    Serial.print("yaw\t");
    Serial.println(rpy[2]);
    lastyaw = rpy[2];
  }
}
//伪串口1中断 处理传感器发过来的数据
void serialEvent1()
{
  while (Serial1.available())
  {
    U8 rxByte = Serial1.read();      // 读取串口的数据
    Cmd_GetPkt(rxByte);           // 移植 每收到1字节数据都填入该函数，当抓取到有效的数据包就会回调进入 Cmd_RxUnpack(U8 *buf, U8 DLen) 函数处理
  }
}