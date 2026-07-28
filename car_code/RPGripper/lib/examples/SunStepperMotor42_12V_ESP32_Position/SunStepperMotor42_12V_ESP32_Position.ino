#include <Arduino.h>
#include <AccelStepper.h>


//200脉冲/圈 16细分 3200个 三等分 1,066.666 1067 1067 1066
//每秒220个脉冲 14.5秒
 
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
//使用X轴步进引脚
//M4
int EN_PIN = 5;    //使能引脚
#define dirPin 19
#define stepPin 18
#define motorInterfaceType 1
 
// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
 
void setup() {
    pinMode( EN_PIN,  OUTPUT ); 
  digitalWrite(EN_PIN, LOW);    //使能步进电机
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(100);
}
 
void loop() 
{ 
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
  stepper.runToNewPosition(1067);
  
  delay(4000);//初赛四秒
   // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
 stepper.runToNewPosition(1067);
 
  delay(4000);//初赛四秒
     // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
 stepper.runToNewPosition(1066);
  delay(4000);//初赛四秒

}