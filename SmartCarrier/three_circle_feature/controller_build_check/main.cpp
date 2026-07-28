#include <Arduino.h>

#include "../controller/ThreeCircleVision.cpp"
#include "../controller/VisualTaskCoordinator.cpp"
#include "../controller/ArmTaskPlan.h"

HardwareSerial visionSerial(PE7, PE8);
ThreeCircleFeature::ThreeCircleVision vision(visionSerial);
ThreeCircleFeature::VisualTaskCoordinator coordinator(vision);
ThreeCircleFeature::ArmTaskPlan taskPlan;

void setup() {
  vision.begin();
}

void loop() {
  coordinator.update();
  delay(1);
}
