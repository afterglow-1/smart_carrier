#pragma once

#include <Arduino.h>

namespace ThreeCircleFeature {

constexpr uint8_t kVisionPacketSize = 30;
constexpr uint8_t kVisionResultType = 0x31;
constexpr uint8_t kSelectThreeCircleMode = 0xF2;
constexpr uint32_t kVisionBaud = 115200;
constexpr uint32_t kPacketAssemblyTimeoutMs = 100;

enum class TargetSource : uint8_t {
  Missing = 0,
  Measured = 1,
  Inferred = 2,
  Fused = 3,
};

struct Target {
  uint8_t number = 0;
  TargetSource source = TargetSource::Missing;
  int16_t x = 0;
  int16_t y = 0;
  uint8_t confidence = 0;
};

struct Result {
  uint16_t sequence = 0;
  uint8_t flags = 0;
  uint8_t boardQuality = 0;
  Target targets[3];
  uint32_t receivedAtMs = 0;

  bool valid() const { return (flags & 0x01U) != 0U; }
  bool containsInference() const { return (flags & 0x02U) != 0U; }
  bool containsFusion() const { return (flags & 0x04U) != 0U; }
};

class ThreeCircleVision {
 public:
  explicit ThreeCircleVision(HardwareSerial &serial);

  void begin(uint32_t baud = kVisionBaud);
  void selectMode();
  void requestResult();
  void poll();
  void clear();

  bool hasResult() const { return hasResult_; }
  bool hasFreshResult(uint32_t maxAgeMs) const;
  const Result &result() const { return result_; }

 private:
  void consume(uint8_t value);
  bool parsePacket();
  static uint16_t crc16Modbus(const uint8_t *data, size_t length);
  static int16_t readI16(const uint8_t *data);

  HardwareSerial &serial_;
  uint8_t packet_[kVisionPacketSize] = {};
  uint8_t byteCount_ = 0;
  uint32_t lastByteAtMs_ = 0;
  bool hasResult_ = false;
  Result result_;
};

}  // namespace ThreeCircleFeature
