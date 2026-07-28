#include <AccelStepper.h>
//200脉冲/圈 16细分 3200个 三等分 1,066.666 1067 1067 1066
 
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
int EN_PIN = 8;    //使能引脚
#define dirPin 5
#define stepPin 2
#define motorInterfaceType 1
 
// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
 
void setup() {
    pinMode( EN_PIN,  OUTPUT ); 
  digitalWrite(EN_PIN, LOW);    //使能步进电机
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(300);
}
 
void loop() 
{ 
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while(stepper.currentPosition() != 1067)
  {
    stepper.setSpeed(300);
    stepper.run();
  }
 
  delay(4000);//初赛四秒
   // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while(stepper.currentPosition() != 1067)
  {
    stepper.setSpeed(300);
    stepper.run();
  }
 
  delay(4000);//初赛四秒
     // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while(stepper.currentPosition() != 1066)
  {
    stepper.setSpeed(300);
    stepper.runSpeed();
  }
 
  delay(4000);//初赛四秒

}