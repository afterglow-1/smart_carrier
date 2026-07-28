// Quickstop.pde
// -*- mode: C++ -*-
//
// Check stop handling.
// Calls stop() while the stepper is travelling at full speed, causing
// the stepper to stop as quickly as possible, within the constraints of the
// current acceleration.
//
// Copyright (C) 2012 Mike McCauley
// $Id:  $
#include <Arduino.h>
#include <AccelStepper.h>
#include <U8g2lib.h>
const int EN_PIN = 5; // 使能引脚
#define dirPin 19
#define stepPin 18
#define motorInterfaceType 1
// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup()
{
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  stepper.setMaxSpeed(150);
  stepper.setAcceleration(100);
}

void loop()
{
  stepper.moveTo(1000);
  while (stepper.currentPosition() != 500) // Full speed up to 300
    stepper.run();
  stepper.stop(); // Stop as fast as possible: sets new target
  stepper.runToPosition();
  // Now stopped after quickstop

  // Now go backwards
  stepper.moveTo(-1000);
  while (stepper.currentPosition() != 0) // Full speed basck to 0
    stepper.run();
  stepper.stop(); // Stop as fast as possible: sets new target
  stepper.runToPosition();
  // Now stopped after quickstop
}
