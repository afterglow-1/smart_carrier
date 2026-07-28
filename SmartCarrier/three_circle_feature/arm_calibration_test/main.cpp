#include <Arduino.h>
#include <AccelStepper.h>
#include <TTL_STEPPER.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint32_t DEBUG_RX_PIN = PB12;
constexpr uint32_t DEBUG_TX_PIN = PB13;
constexpr uint32_t DEBUG_BAUD = 115200;

constexpr uint32_t ARM_UART_RX_PIN = PA3;
constexpr uint32_t ARM_UART_TX_PIN = PA2;
constexpr uint32_t ARM_UART_BAUD = 115200;
constexpr uint8_t ID6 = 6;
constexpr uint8_t ID7 = 7;

constexpr uint32_t M5_ENABLE_PIN = PE10;
constexpr uint32_t M5_DIR_PIN = PE15;
constexpr uint32_t M5_STEP_PIN = PB11;
constexpr uint8_t MOTOR_INTERFACE_TYPE = AccelStepper::DRIVER;
constexpr float M5_GEAR_RATIO = 5.0F;
constexpr float M5_MOTOR_PULSES_PER_REV = 200.0F * 16.0F;
constexpr float M5_PULSES_PER_OUTPUT_DEGREE =
    M5_MOTOR_PULSES_PER_REV * M5_GEAR_RATIO / 360.0F;
constexpr float M5_MAX_SPEED_PPS = 1000.0F;
constexpr float M5_ACCELERATION_PPS2 = 500.0F;
constexpr int8_t M5_CW_SIGN = -1;
constexpr float M5_MIN_CW_DEG = 0.0F;
constexpr float M5_MAX_CW_DEG = 140.0F;

constexpr uint8_t SENSORLESS_HOME_MODE = 2;
constexpr uint32_t QUERY_TIMEOUT_MS = 120;
constexpr uint32_t HOME_POLL_INTERVAL_MS = 250;
constexpr uint32_t HOME_MONITOR_TIMEOUT_MS = 60000;
constexpr uint8_t MAX_QUERY_FAILURES = 5;
constexpr uint32_t MAX_JOG_PULSES = 3200;
constexpr uint16_t DEFAULT_JOG_RPM = 60;
constexpr uint8_t DEFAULT_JOG_ACCELERATION = 100;

constexpr float ID6_NOMINAL_MM_PER_MOTOR_REV = PI * 1.0F * 36.0F;
constexpr float ID7_NOMINAL_MM_PER_MOTOR_REV = 12.0F;

HardwareSerial SerialDebug(DEBUG_RX_PIN, DEBUG_TX_PIN);
HardwareSerial SerialArm(ARM_UART_RX_PIN, ARM_UART_TX_PIN);
TTL_Protocol armProtocol(&SerialArm, ARM_UART_BAUD);
AccelStepper m5(MOTOR_INTERFACE_TYPE, M5_STEP_PIN, M5_DIR_PIN);

struct PositionReading {
  bool valid = false;
  bool negative = false;
  uint32_t magnitude = 0;
};

struct HomeMonitor {
  bool active = false;
  bool sawRunning = false;
  bool warnedNoRunningFlag = false;
  bool hasFlags = false;
  uint8_t id = 0;
  uint8_t lastFlags = 0;
  uint8_t queryFailures = 0;
  uint32_t startedMs = 0;
  uint32_t lastPollMs = 0;
};

HomeMonitor homeMonitor;
char commandLine[128] = {};
size_t commandLength = 0;
bool m5MotionActive = false;

bool isArmId(uint8_t id) {
  return id == ID6 || id == ID7;
}

bool parseArmId(const char *text, uint8_t &id) {
  if (text == nullptr) {
    return false;
  }
  const long value = strtol(text, nullptr, 10);
  if (value != ID6 && value != ID7) {
    return false;
  }
  id = static_cast<uint8_t>(value);
  return true;
}

bool parseUnsignedLong(const char *text, unsigned long &value) {
  if (text == nullptr || *text == '\0' || *text == '-') {
    return false;
  }
  char *end = nullptr;
  value = strtoul(text, &end, 10);
  return end != text && *end == '\0';
}

bool parseFloat(const char *text, float &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

bool equalsIgnoreCase(const char *left, const char *right) {
  if (left == nullptr || right == nullptr) {
    return false;
  }
  while (*left != '\0' && *right != '\0') {
    if (tolower(static_cast<unsigned char>(*left)) !=
        tolower(static_cast<unsigned char>(*right))) {
      return false;
    }
    ++left;
    ++right;
  }
  return *left == '\0' && *right == '\0';
}

void drainArmRx() {
  while (SerialArm.available() > 0) {
    SerialArm.read();
  }
}

bool readFrame(uint8_t id,
               uint8_t function,
               uint8_t expectedLength,
               uint8_t *frame,
               uint32_t timeoutMs) {
  uint8_t window[24] = {};
  uint8_t count = 0;
  const uint32_t startedMs = millis();

  while (millis() - startedMs < timeoutMs) {
    while (SerialArm.available() > 0) {
      const uint8_t incoming = static_cast<uint8_t>(SerialArm.read());
      if (count < expectedLength) {
        window[count++] = incoming;
      } else {
        memmove(window, window + 1, expectedLength - 1);
        window[expectedLength - 1] = incoming;
      }

      if (count == expectedLength &&
          window[0] == id &&
          window[1] == function &&
          window[expectedLength - 1] == 0x6B) {
        memcpy(frame, window, expectedLength);
        return true;
      }
    }
    delay(1);
  }
  return false;
}

bool queryShortFlags(uint8_t id, SysParams_t parameter, uint8_t &flags) {
  uint8_t frame[4] = {};
  drainArmRx();
  armProtocol.Emm_V5_Read_Sys_Params(id, parameter);
  if (!readFrame(id,
                 parameter == S_FLAG ? 0x3A : 0x3B,
                 sizeof(frame),
                 frame,
                 QUERY_TIMEOUT_MS)) {
    return false;
  }
  flags = frame[2];
  return true;
}

bool queryPosition(uint8_t id, PositionReading &position) {
  uint8_t frame[8] = {};
  drainArmRx();
  armProtocol.Emm_V5_Read_Sys_Params(id, S_CPOS);
  if (!readFrame(id, 0x36, sizeof(frame), frame, QUERY_TIMEOUT_MS)) {
    position.valid = false;
    return false;
  }

  position.valid = true;
  position.negative = frame[2] != 0;
  position.magnitude =
      (static_cast<uint32_t>(frame[3]) << 24) |
      (static_cast<uint32_t>(frame[4]) << 16) |
      (static_cast<uint32_t>(frame[5]) << 8) |
      static_cast<uint32_t>(frame[6]);
  return true;
}

uint16_t readU16(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) |
         static_cast<uint16_t>(data[1]);
}

uint32_t readU32(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    SerialDebug.print('0');
  }
  SerialDebug.print(value, HEX);
}

void printSignedMagnitude(const PositionReading &position) {
  if (!position.valid) {
    SerialDebug.print(F("NA"));
    return;
  }
  if (position.negative && position.magnitude != 0) {
    SerialDebug.print('-');
  }
  SerialDebug.print(static_cast<unsigned long>(position.magnitude));
}

double positionMotorDegrees(const PositionReading &position) {
  if (!position.valid) {
    return 0.0;
  }
  const double sign = position.negative ? -1.0 : 1.0;
  return sign * static_cast<double>(position.magnitude) * 360.0 / 65536.0;
}

double positionNominalMm(const PositionReading &position, uint8_t id) {
  if (!position.valid) {
    return 0.0;
  }
  const double sign = position.negative ? -1.0 : 1.0;
  const double millimetersPerRevolution =
      id == ID6 ? ID6_NOMINAL_MM_PER_MOTOR_REV
                : ID7_NOMINAL_MM_PER_MOTOR_REV;
  return sign * static_cast<double>(position.magnitude) /
         65536.0 * millimetersPerRevolution;
}

void printPosition(uint8_t id) {
  PositionReading position;
  SerialDebug.print(F("POS,id="));
  SerialDebug.print(id);
  if (!queryPosition(id, position)) {
    SerialDebug.println(F(",error=timeout"));
    return;
  }
  SerialDebug.print(F(",raw="));
  printSignedMagnitude(position);
  SerialDebug.print(F(",motor_deg="));
  SerialDebug.print(positionMotorDegrees(position), 4);
  SerialDebug.print(F(",nominal_mm="));
  SerialDebug.println(positionNominalMm(position, id), 4);
}

void printStateFlags(uint8_t id) {
  uint8_t flags = 0;
  SerialDebug.print(F("STATE,id="));
  SerialDebug.print(id);
  if (!queryShortFlags(id, S_FLAG, flags)) {
    SerialDebug.println(F(",error=timeout"));
    return;
  }
  SerialDebug.print(F(",flags=0x"));
  printHexByte(flags);
  SerialDebug.print(F(",enabled="));
  SerialDebug.print((flags & 0x01) != 0);
  SerialDebug.print(F(",on_position="));
  SerialDebug.print((flags & 0x02) != 0);
  SerialDebug.print(F(",stalled="));
  SerialDebug.print((flags & 0x04) != 0);
  SerialDebug.print(F(",stall_protection="));
  SerialDebug.println((flags & 0x08) != 0);
}

void printOriginFlags(uint8_t id, uint8_t flags) {
  SerialDebug.print(F("ORIGIN,id="));
  SerialDebug.print(id);
  SerialDebug.print(F(",flags=0x"));
  printHexByte(flags);
  SerialDebug.print(F(",encoder_ready="));
  SerialDebug.print((flags & 0x01) != 0);
  SerialDebug.print(F(",calibration_ready="));
  SerialDebug.print((flags & 0x02) != 0);
  SerialDebug.print(F(",homing="));
  SerialDebug.print((flags & 0x04) != 0);
  SerialDebug.print(F(",homing_failed="));
  SerialDebug.println((flags & 0x08) != 0);
}

void queryAndPrintOriginFlags(uint8_t id) {
  uint8_t flags = 0;
  if (!queryShortFlags(id, S_ORG, flags)) {
    SerialDebug.print(F("ORIGIN,id="));
    SerialDebug.print(id);
    SerialDebug.println(F(",error=timeout"));
    return;
  }
  printOriginFlags(id, flags);
}

void printOriginParameters(uint8_t id) {
  constexpr uint8_t responseLength = 18;
  uint8_t frame[responseLength] = {};
  const uint8_t command[] = {id, 0x22, 0x6B};
  drainArmRx();
  SerialArm.write(command, sizeof(command));
  if (!readFrame(id,
                 0x22,
                 responseLength,
                 frame,
                 QUERY_TIMEOUT_MS)) {
    SerialDebug.print(F("PARAMS,id="));
    SerialDebug.print(id);
    SerialDebug.println(F(",error=timeout"));
    return;
  }

  SerialDebug.print(F("PARAMS,id="));
  SerialDebug.print(id);
  SerialDebug.print(F(",mode="));
  SerialDebug.print(frame[2]);
  SerialDebug.print(F(",direction="));
  SerialDebug.print(frame[3] == 0 ? F("CW") : F("CCW"));
  SerialDebug.print(F(",home_rpm="));
  SerialDebug.print(readU16(frame + 4));
  SerialDebug.print(F(",timeout_ms="));
  SerialDebug.print(static_cast<unsigned long>(readU32(frame + 6)));
  SerialDebug.print(F(",sensorless_rpm="));
  SerialDebug.print(readU16(frame + 10));
  SerialDebug.print(F(",sensorless_ma="));
  SerialDebug.print(readU16(frame + 12));
  SerialDebug.print(F(",sensorless_ms="));
  SerialDebug.print(readU16(frame + 14));
  SerialDebug.print(F(",power_on_home="));
  SerialDebug.println(frame[16] != 0);
}

void enableAxis(uint8_t id, bool enabled) {
  drainArmRx();
  armProtocol.Emm_V5_En_Control(id, enabled, false);
  SerialDebug.print(F("AXIS,id="));
  SerialDebug.print(id);
  SerialDebug.print(F(",enable_command="));
  SerialDebug.println(enabled ? 1 : 0);
}

void stopAxis(uint8_t id) {
  armProtocol.Emm_V5_Origin_Interrupt(id);
  delay(5);
  armProtocol.Emm_V5_Stop_Now(id, false);
  if (homeMonitor.active && homeMonitor.id == id) {
    homeMonitor.active = false;
  }
  SerialDebug.print(F("STOP,id="));
  SerialDebug.println(id);
}

void emergencyStop() {
  m5.moveTo(m5.currentPosition());
  digitalWrite(M5_ENABLE_PIN, HIGH);
  m5MotionActive = false;
  armProtocol.Emm_V5_Origin_Interrupt(ID6);
  armProtocol.Emm_V5_Origin_Interrupt(ID7);
  delay(5);
  armProtocol.Emm_V5_Stop_Now(ID6, false);
  armProtocol.Emm_V5_Stop_Now(ID7, false);
  homeMonitor.active = false;
  SerialDebug.println(F("EMERGENCY_STOP,all_axes_commanded_to_stop"));
}

void startSensorlessHome(uint8_t id) {
  if (homeMonitor.active) {
    SerialDebug.print(F("ERROR,home_already_active,id="));
    SerialDebug.println(homeMonitor.id);
    return;
  }
  if (m5MotionActive) {
    SerialDebug.println(F("ERROR,home_rejected_while_m5_is_moving"));
    return;
  }

  enableAxis(id, true);
  delay(20);
  drainArmRx();
  armProtocol.Emm_V5_Origin_Trigger_Return(
      id, SENSORLESS_HOME_MODE, false);

  homeMonitor = HomeMonitor{};
  homeMonitor.active = true;
  homeMonitor.id = id;
  homeMonitor.startedMs = millis();
  SerialDebug.print(F("HOME_START,id="));
  SerialDebug.print(id);
  SerialDebug.println(F(",mode=2,saved_driver_parameters_unchanged"));
}

void updateHomeMonitor() {
  if (!homeMonitor.active) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - homeMonitor.startedMs >= HOME_MONITOR_TIMEOUT_MS) {
    SerialDebug.print(F("HOME_MONITOR_TIMEOUT,id="));
    SerialDebug.println(homeMonitor.id);
    stopAxis(homeMonitor.id);
    return;
  }
  if (nowMs - homeMonitor.lastPollMs < HOME_POLL_INTERVAL_MS) {
    return;
  }
  homeMonitor.lastPollMs = nowMs;

  uint8_t flags = 0;
  if (!queryShortFlags(homeMonitor.id, S_ORG, flags)) {
    ++homeMonitor.queryFailures;
    SerialDebug.print(F("HOME_QUERY_TIMEOUT,id="));
    SerialDebug.print(homeMonitor.id);
    SerialDebug.print(F(",count="));
    SerialDebug.println(homeMonitor.queryFailures);
    if (homeMonitor.queryFailures >= MAX_QUERY_FAILURES) {
      SerialDebug.println(F("HOME_ABORT,too_many_query_failures"));
      stopAxis(homeMonitor.id);
    }
    return;
  }

  homeMonitor.queryFailures = 0;
  if (!homeMonitor.hasFlags || flags != homeMonitor.lastFlags) {
    printOriginFlags(homeMonitor.id, flags);
    homeMonitor.lastFlags = flags;
    homeMonitor.hasFlags = true;
  }

  const bool running = (flags & 0x04) != 0;
  const bool failed = (flags & 0x08) != 0;
  if (running) {
    homeMonitor.sawRunning = true;
  }
  if (failed) {
    SerialDebug.print(F("HOME_FAILED,id="));
    SerialDebug.println(homeMonitor.id);
    homeMonitor.active = false;
    return;
  }
  if (homeMonitor.sawRunning && !running) {
    const uint8_t completedId = homeMonitor.id;
    homeMonitor.active = false;
    SerialDebug.print(F("HOME_COMPLETE,id="));
    SerialDebug.println(completedId);
    printPosition(completedId);
    return;
  }
  if (!homeMonitor.sawRunning &&
      !homeMonitor.warnedNoRunningFlag &&
      nowMs - homeMonitor.startedMs >= 3000) {
    homeMonitor.warnedNoRunningFlag = true;
    SerialDebug.println(
        F("HOME_WARNING,no_running_flag_seen; verify driver mode and direction"));
  }
}

void jogAxis(uint8_t id,
             bool clockwise,
             uint32_t pulses,
             uint16_t rpm,
             uint8_t acceleration) {
  if (homeMonitor.active) {
    SerialDebug.println(F("ERROR,jog_rejected_while_homing"));
    return;
  }
  if (m5MotionActive) {
    SerialDebug.println(F("ERROR,jog_rejected_while_m5_is_moving"));
    return;
  }
  if (pulses == 0 || pulses > MAX_JOG_PULSES) {
    SerialDebug.print(F("ERROR,pulses_must_be_1_to_"));
    SerialDebug.println(static_cast<unsigned long>(MAX_JOG_PULSES));
    return;
  }
  if (rpm == 0 || rpm > 300) {
    SerialDebug.println(F("ERROR,rpm_must_be_1_to_300"));
    return;
  }

  enableAxis(id, true);
  delay(20);
  drainArmRx();
  armProtocol.Emm_V5_Pos_Control(
      id,
      clockwise ? 0 : 1,
      rpm,
      acceleration,
      pulses,
      false,
      false);
  SerialDebug.print(F("JOG,id="));
  SerialDebug.print(id);
  SerialDebug.print(F(",direction="));
  SerialDebug.print(clockwise ? F("CW") : F("CCW"));
  SerialDebug.print(F(",pulses="));
  SerialDebug.print(static_cast<unsigned long>(pulses));
  SerialDebug.print(F(",rpm="));
  SerialDebug.print(rpm);
  SerialDebug.print(F(",acceleration="));
  SerialDebug.println(acceleration);
}

float m5CurrentCwDegrees() {
  return static_cast<float>(m5.currentPosition()) /
         (static_cast<float>(M5_CW_SIGN) * M5_PULSES_PER_OUTPUT_DEGREE);
}

void printM5Status() {
  SerialDebug.print(F("M5,pulses="));
  SerialDebug.print(m5.currentPosition());
  SerialDebug.print(F(",cw_deg="));
  SerialDebug.print(m5CurrentCwDegrees(), 3);
  SerialDebug.print(F(",target_pulses="));
  SerialDebug.print(m5.targetPosition());
  SerialDebug.print(F(",moving="));
  SerialDebug.println(m5.distanceToGo() != 0);
}

void moveM5To(float targetCwDegrees) {
  if (homeMonitor.active) {
    SerialDebug.println(F("ERROR,m5_move_rejected_while_homing"));
    return;
  }
  if (targetCwDegrees < M5_MIN_CW_DEG ||
      targetCwDegrees > M5_MAX_CW_DEG) {
    SerialDebug.print(F("ERROR,m5_target_range_deg="));
    SerialDebug.print(M5_MIN_CW_DEG, 0);
    SerialDebug.print(F(".."));
    SerialDebug.println(M5_MAX_CW_DEG, 0);
    return;
  }
  const long targetPulses = lroundf(
      targetCwDegrees * static_cast<float>(M5_CW_SIGN) *
      M5_PULSES_PER_OUTPUT_DEGREE);
  digitalWrite(M5_ENABLE_PIN, LOW);
  m5.moveTo(targetPulses);
  m5MotionActive = true;
  SerialDebug.print(F("M5_MOVE,target_cw_deg="));
  SerialDebug.print(targetCwDegrees, 3);
  SerialDebug.print(F(",target_pulses="));
  SerialDebug.println(targetPulses);
}

void updateM5() {
  if (!m5MotionActive) {
    return;
  }
  m5.run();
  if (m5.distanceToGo() == 0) {
    m5MotionActive = false;
    SerialDebug.println(F("M5_COMPLETE"));
    printM5Status();
  }
}

void printCsvHeader() {
  SerialDebug.println(
      F("MARK_HEADER,time_ms,label,m5_pulses,m5_cw_deg,"
        "id6_raw,id6_motor_deg,id6_nominal_mm,id6_state,"
        "id7_raw,id7_motor_deg,id7_nominal_mm,id7_state"));
}

void printPositionCsv(const PositionReading &position, uint8_t id) {
  printSignedMagnitude(position);
  SerialDebug.print(',');
  if (position.valid) {
    SerialDebug.print(positionMotorDegrees(position), 4);
  } else {
    SerialDebug.print(F("NA"));
  }
  SerialDebug.print(',');
  if (position.valid) {
    SerialDebug.print(positionNominalMm(position, id), 4);
  } else {
    SerialDebug.print(F("NA"));
  }
}

void markPose(const char *label) {
  if (label == nullptr || *label == '\0') {
    SerialDebug.println(F("ERROR,mark_requires_a_label"));
    return;
  }
  for (const char *p = label; *p != '\0'; ++p) {
    if (!isalnum(static_cast<unsigned char>(*p)) &&
        *p != '_' && *p != '-') {
      SerialDebug.println(
          F("ERROR,label_use_only_letters_digits_underscore_hyphen"));
      return;
    }
  }
  if (m5MotionActive || homeMonitor.active) {
    SerialDebug.println(F("ERROR,mark_requires_all_axes_stopped"));
    return;
  }

  PositionReading id6Position;
  PositionReading id7Position;
  uint8_t id6State = 0;
  uint8_t id7State = 0;
  queryPosition(ID6, id6Position);
  queryShortFlags(ID6, S_FLAG, id6State);
  queryPosition(ID7, id7Position);
  queryShortFlags(ID7, S_FLAG, id7State);

  SerialDebug.print(F("MARK,"));
  SerialDebug.print(millis());
  SerialDebug.print(',');
  SerialDebug.print(label);
  SerialDebug.print(',');
  SerialDebug.print(m5.currentPosition());
  SerialDebug.print(',');
  SerialDebug.print(m5CurrentCwDegrees(), 3);
  SerialDebug.print(',');
  printPositionCsv(id6Position, ID6);
  SerialDebug.print(',');
  SerialDebug.print(F("0x"));
  printHexByte(id6State);
  SerialDebug.print(',');
  printPositionCsv(id7Position, ID7);
  SerialDebug.print(',');
  SerialDebug.print(F("0x"));
  printHexByte(id7State);
  SerialDebug.println();
}

void printHelp() {
  SerialDebug.println(F("Commands:"));
  SerialDebug.println(F("  help"));
  SerialDebug.println(F("  params <6|7>             read saved homing parameters"));
  SerialDebug.println(F("  home <6|7>               trigger mode-2 sensorless homing"));
  SerialDebug.println(F("  origin <6|7|all>         read homing flags"));
  SerialDebug.println(F("  abort <6|7>              interrupt homing and stop axis"));
  SerialDebug.println(F("  enable <6|7> | disable <6|7>"));
  SerialDebug.println(F("  jog <6|7> <cw|ccw> <pulses> [rpm] [acc]"));
  SerialDebug.println(F("  pos <6|7|all> | state <6|7|all>"));
  SerialDebug.println(F("  m5 zero                  current physical pose = 0 deg"));
  SerialDebug.println(F("  m5 goto <0..140>         absolute clockwise output angle"));
  SerialDebug.println(F("  m5 jog <signed_deg>      relative clockwise angle"));
  SerialDebug.println(F("  m5 status | m5 stop | m5 disable"));
  SerialDebug.println(F("  csv | mark <label>       emit calibration CSV row"));
  SerialDebug.println(F("  stop all                 stop M5, ID6, and ID7"));
  SerialDebug.println(F("  !                        immediate emergency stop"));
}

void processAxisListCommand(const char *axis,
                            void (*operation)(uint8_t)) {
  if (axis != nullptr && equalsIgnoreCase(axis, "all")) {
    operation(ID6);
    operation(ID7);
    return;
  }
  uint8_t id = 0;
  if (!parseArmId(axis, id)) {
    SerialDebug.println(F("ERROR,axis_must_be_6_7_or_all"));
    return;
  }
  operation(id);
}

void processCommand(char *line) {
  char *command = strtok(line, " \t");
  if (command == nullptr) {
    return;
  }

  if (equalsIgnoreCase(command, "help") || strcmp(command, "?") == 0) {
    printHelp();
    return;
  }
  if (equalsIgnoreCase(command, "params")) {
    uint8_t id = 0;
    if (!parseArmId(strtok(nullptr, " \t"), id)) {
      SerialDebug.println(F("ERROR,params_requires_6_or_7"));
      return;
    }
    printOriginParameters(id);
    return;
  }
  if (equalsIgnoreCase(command, "home")) {
    uint8_t id = 0;
    if (!parseArmId(strtok(nullptr, " \t"), id)) {
      SerialDebug.println(F("ERROR,home_requires_6_or_7"));
      return;
    }
    startSensorlessHome(id);
    return;
  }
  if (equalsIgnoreCase(command, "origin")) {
    processAxisListCommand(strtok(nullptr, " \t"), queryAndPrintOriginFlags);
    return;
  }
  if (equalsIgnoreCase(command, "pos")) {
    processAxisListCommand(strtok(nullptr, " \t"), printPosition);
    return;
  }
  if (equalsIgnoreCase(command, "state")) {
    processAxisListCommand(strtok(nullptr, " \t"), printStateFlags);
    return;
  }
  if (equalsIgnoreCase(command, "abort")) {
    uint8_t id = 0;
    if (!parseArmId(strtok(nullptr, " \t"), id)) {
      SerialDebug.println(F("ERROR,abort_requires_6_or_7"));
      return;
    }
    stopAxis(id);
    return;
  }
  if (equalsIgnoreCase(command, "enable") ||
      equalsIgnoreCase(command, "disable")) {
    uint8_t id = 0;
    if (!parseArmId(strtok(nullptr, " \t"), id)) {
      SerialDebug.println(F("ERROR,enable_disable_requires_6_or_7"));
      return;
    }
    enableAxis(id, equalsIgnoreCase(command, "enable"));
    return;
  }
  if (equalsIgnoreCase(command, "jog")) {
    uint8_t id = 0;
    char *axis = strtok(nullptr, " \t");
    char *direction = strtok(nullptr, " \t");
    char *pulseText = strtok(nullptr, " \t");
    char *rpmText = strtok(nullptr, " \t");
    char *accelerationText = strtok(nullptr, " \t");
    if (!parseArmId(axis, id) || direction == nullptr || pulseText == nullptr) {
      SerialDebug.println(
          F("ERROR,use_jog_id_cw_or_ccw_pulses_optional_rpm_acc"));
      return;
    }
    const bool clockwise = equalsIgnoreCase(direction, "cw");
    if (!clockwise && !equalsIgnoreCase(direction, "ccw")) {
      SerialDebug.println(F("ERROR,direction_must_be_cw_or_ccw"));
      return;
    }
    unsigned long pulses = 0;
    unsigned long rpm = DEFAULT_JOG_RPM;
    unsigned long acceleration = DEFAULT_JOG_ACCELERATION;
    if (!parseUnsignedLong(pulseText, pulses) ||
        (rpmText != nullptr && !parseUnsignedLong(rpmText, rpm)) ||
        (accelerationText != nullptr &&
         !parseUnsignedLong(accelerationText, acceleration)) ||
        rpm > 300 || acceleration > 255) {
      SerialDebug.println(F("ERROR,invalid_jog_numeric_parameter"));
      return;
    }
    jogAxis(id,
            clockwise,
            static_cast<uint32_t>(pulses),
            static_cast<uint16_t>(rpm),
            static_cast<uint8_t>(acceleration));
    return;
  }
  if (equalsIgnoreCase(command, "m5")) {
    char *action = strtok(nullptr, " \t");
    if (equalsIgnoreCase(action, "zero")) {
      if (m5.distanceToGo() != 0) {
        SerialDebug.println(F("ERROR,m5_must_be_stopped_before_zero"));
        return;
      }
      m5.setCurrentPosition(0);
      m5.moveTo(0);
      SerialDebug.println(F("M5_SOFTWARE_ZERO_SET"));
      return;
    }
    if (equalsIgnoreCase(action, "goto")) {
      char *angleText = strtok(nullptr, " \t");
      float angle = 0.0F;
      if (!parseFloat(angleText, angle)) {
        SerialDebug.println(F("ERROR,m5_goto_requires_degrees"));
        return;
      }
      moveM5To(angle);
      return;
    }
    if (equalsIgnoreCase(action, "jog")) {
      char *angleText = strtok(nullptr, " \t");
      float angle = 0.0F;
      if (!parseFloat(angleText, angle)) {
        SerialDebug.println(F("ERROR,m5_jog_requires_signed_degrees"));
        return;
      }
      moveM5To(m5CurrentCwDegrees() + angle);
      return;
    }
    if (equalsIgnoreCase(action, "status")) {
      printM5Status();
      return;
    }
    if (equalsIgnoreCase(action, "stop")) {
      m5.stop();
      m5MotionActive = m5.distanceToGo() != 0;
      SerialDebug.println(F("M5_CONTROLLED_STOP"));
      return;
    }
    if (equalsIgnoreCase(action, "disable")) {
      m5.moveTo(m5.currentPosition());
      m5MotionActive = false;
      digitalWrite(M5_ENABLE_PIN, HIGH);
      SerialDebug.println(
          F("M5_DISABLED,software_position_invalid_if_axis_is_moved_by_hand"));
      return;
    }
    SerialDebug.println(F("ERROR,m5_action_zero_goto_jog_status_stop_disable"));
    return;
  }
  if (equalsIgnoreCase(command, "csv")) {
    printCsvHeader();
    return;
  }
  if (equalsIgnoreCase(command, "mark")) {
    markPose(strtok(nullptr, " \t"));
    return;
  }
  if (equalsIgnoreCase(command, "stop")) {
    char *target = strtok(nullptr, " \t");
    if (target != nullptr && equalsIgnoreCase(target, "all")) {
      emergencyStop();
      return;
    }
    SerialDebug.println(F("ERROR,use_stop_all_or_exclamation_mark"));
    return;
  }

  SerialDebug.println(F("ERROR,unknown_command; type_help"));
}

void updateCommandInput() {
  while (SerialDebug.available() > 0) {
    const char incoming = static_cast<char>(SerialDebug.read());
    if (incoming == '!') {
      commandLength = 0;
      commandLine[0] = '\0';
      emergencyStop();
      continue;
    }
    if (incoming == '\r' || incoming == '\n') {
      if (commandLength > 0) {
        commandLine[commandLength] = '\0';
        processCommand(commandLine);
        commandLength = 0;
        commandLine[0] = '\0';
      }
      continue;
    }
    if (isprint(static_cast<unsigned char>(incoming)) &&
        commandLength < sizeof(commandLine) - 1) {
      commandLine[commandLength++] = incoming;
    }
  }
}

}  // namespace

void setup() {
  pinMode(M5_ENABLE_PIN, OUTPUT);
  digitalWrite(M5_ENABLE_PIN, HIGH);

  SerialDebug.begin(DEBUG_BAUD);
  SerialArm.begin(ARM_UART_BAUD);
  armProtocol.init(&SerialArm, ARM_UART_BAUD);

  m5.setMaxSpeed(M5_MAX_SPEED_PPS);
  m5.setAcceleration(M5_ACCELERATION_PPS2);
  m5.setCurrentPosition(0);
  m5.moveTo(0);

  delay(300);
  SerialDebug.println();
  SerialDebug.println(F("ARM_CALIBRATION_TEST_READY"));
  SerialDebug.println(
      F("M5 startup physical pose is software zero; gear ratio=5:1."));
  SerialDebug.println(
      F("ID6/ID7 homing command uses mode 2 and keeps saved parameters."));
  SerialDebug.println(F("Send ! at any time for emergency stop."));
  printHelp();
  printCsvHeader();
}

void loop() {
  updateCommandInput();
  updateM5();
  updateHomeMonitor();
  delay(1);
}
