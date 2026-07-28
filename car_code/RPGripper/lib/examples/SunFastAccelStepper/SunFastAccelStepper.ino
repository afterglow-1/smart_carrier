#include <Arduino.h>
#include "FastAccelStepper.h"

//使用闭环步进 


//200脉冲/圈 16细分 3200个 三等分 1,066.666 1067 1067 1066
//每秒220个脉冲 14.5秒
 
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
//使用X轴步进引脚
//M4

#define dirPinStepper 19
#define enablePinStepper 5
#define stepPinStepper 18 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInUs(10);  // the parameter is us/step !!!
    stepper->setAcceleration(100);
    //stepper->move(1066);
    
  }
}

void loop() {
stepper->setPositionAfterCommandsCompleted(0);
stepper->move(1066,true);
delay(4000);
stepper->setPositionAfterCommandsCompleted(0);
stepper->move(1066,true);
delay(4000);
stepper->setPositionAfterCommandsCompleted(0);
stepper->move(1067,true);
delay(4000);

}

