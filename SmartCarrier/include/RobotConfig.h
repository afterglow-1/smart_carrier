#pragma once

#include <Arduino.h>

namespace Config {

constexpr uint32_t DEBUG_BAUD = 115200;

// Four mecanum-wheel step/direction drivers (from final/src/main.cpp).
constexpr uint32_t DRIVE_ENABLE_PIN = PE13;
constexpr uint32_t DRIVE_DIR_PINS[4] = {PD6, PE9, PD14, PC3_C};
constexpr uint32_t DRIVE_STEP_PINS[4] = {PD4, PE11, PD15, PA1};
constexpr bool DRIVE_ENABLE_LEVEL = LOW;
constexpr bool DRIVE_DISABLE_LEVEL = HIGH;

// Mechanical-arm base rotation step/direction driver.
constexpr uint32_t ROTATE_ENABLE_PIN = PE10;
constexpr uint32_t ROTATE_DIR_PIN = PE15;
constexpr uint32_t ROTATE_STEP_PIN = PB11;
constexpr bool ROTATE_ENABLE_LEVEL = LOW;
constexpr bool ROTATE_DISABLE_LEVEL = HIGH;

// Shared serial bus for closed-loop arm steppers.
constexpr uint32_t ARM_STEPPER_RX = PA3;
constexpr uint32_t ARM_STEPPER_TX = PA2;
constexpr uint32_t ARM_STEPPER_BAUD = 115200;
constexpr uint8_t ARM_STEPPER_IDS[2] = {6, 7};

// FashionStar UART servo bus.
constexpr uint32_t SERVO_RX = PC7;
constexpr uint32_t SERVO_TX = PC6;
constexpr uint32_t SERVO_BAUD = 115200;

// Competition start button. The electrical baseline document specifies PB8.
constexpr uint32_t START_BUTTON_PIN = PB8;

// Conservative commissioning limits. These are deliberately much slower than
// the legacy final program and must be calibrated on the raised vehicle.
constexpr float DRIVE_MAX_SPEED = 1200.0F;
constexpr float DRIVE_ACCELERATION = 600.0F;
constexpr long DRIVE_DEFAULT_JOG_PULSES = 200;
constexpr long DRIVE_MAX_JOG_PULSES = 1000;

constexpr float ROTATE_MAX_SPEED = 400.0F;
constexpr float ROTATE_ACCELERATION = 200.0F;
constexpr long ROTATE_DEFAULT_JOG_PULSES = 50;
constexpr long ROTATE_MAX_JOG_PULSES = 300;

constexpr uint16_t ARM_JOG_RPM = 30;
constexpr uint8_t ARM_JOG_ACCELERATION = 20;
constexpr uint32_t ARM_DEFAULT_JOG_PULSES = 100;
constexpr uint32_t ARM_MAX_JOG_PULSES = 400;

constexpr int SERVO_SCAN_MIN_ID = 0;
constexpr int SERVO_SCAN_MAX_ID = 20;
constexpr float GRIPPER_MIN_ANGLE = -45.0F;
constexpr float GRIPPER_MAX_ANGLE = 45.0F;

constexpr size_t COMMAND_BUFFER_SIZE = 96;

}  // namespace Config
