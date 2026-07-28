#include <AccelStepper.h>
//使用Makerbase MKS Gen V1.4 主板 
//https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-23356668283.19.432c2ca6aXBzh3&id=40156384472
//准备改用Makerbase MKS Gen-L  V2.1
//https://item.taobao.com/item.htm?spm=a1z0d.7625083.1998302264.6.5c5f4e696Apofu&id=543608919100

//200脉冲/圈 16细分 3200个 三等分 1,066.666 1067 1067 1066
//每秒220个脉冲 14.5秒
 
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
//使用X轴步进引脚
int EN_PIN = 38;    //使能引脚
#define dirPin 55
#define stepPin 54
#define motorInterfaceType 1
 
// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
 
void setup() {
    pinMode( EN_PIN,  OUTPUT ); 
  digitalWrite(EN_PIN, LOW);    //使能步进电机 低电平有效
  
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(220);
  stepper.setAcceleration(0.2);
}
 
void loop() 
{ 
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while(stepper.currentPosition() != 1067)
  {
    stepper.setSpeed(220);
    stepper.runSpeed();
  }
 
  delay(4000);//初赛四秒
   // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while(stepper.currentPosition() != 1067)
  {
    stepper.setSpeed(220);
    stepper.runSpeed();
  }
 
  delay(4000);//初赛四秒
     // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while(stepper.currentPosition() != 1066)
  {
    stepper.setSpeed(220);
    stepper.runSpeed();
  }
 
  delay(4000);//初赛四秒

}