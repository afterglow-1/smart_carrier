#include "ThreeCircleVision.h"

namespace ThreeCircleFeature {

ThreeCircleVision::ThreeCircleVision(HardwareSerial &serial) : serial_(serial) {}

void ThreeCircleVision::begin(uint32_t baud) {
  serial_.begin(baud);
  clear();
}

void ThreeCircleVision::selectMode() {
  const uint8_t command[] = {0xAA, 0x00, kSelectThreeCircleMode, 0xBB};
  serial_.write(command, sizeof(command));
  serial_.flush();
}

void ThreeCircleVision::requestResult() {
  const uint8_t command[] = {0xAA, 0x00, kVisionResultType, 0xBB};
  serial_.write(command, sizeof(command));
  serial_.flush();
}

void ThreeCircleVision::poll() {
  if (byteCount_ > 0 && millis() - lastByteAtMs_ > kPacketAssemblyTimeoutMs) {
    byteCount_ = 0;
  }
  while (serial_.available() > 0) {
    consume(static_cast<uint8_t>(serial_.read()));
  }
}

void ThreeCircleVision::clear() {
  byteCount_ = 0;
  hasResult_ = false;
  result_ = Result{};
}

bool ThreeCircleVision::hasFreshResult(uint32_t maxAgeMs) const {
  return hasResult_ && result_.valid() &&
         millis() - result_.receivedAtMs <= maxAgeMs;
}

void ThreeCircleVision::consume(uint8_t value) {
  lastByteAtMs_ = millis();
  if (byteCount_ == 0 && value != 0xAA) {
    return;
  }
  packet_[byteCount_++] = value;
  if (byteCount_ < kVisionPacketSize) {
    return;
  }
  parsePacket();
  byteCount_ = 0;
}

bool ThreeCircleVision::parsePacket() {
  if (packet_[0] != 0xAA || packet_[1] != kVisionResultType ||
      packet_[kVisionPacketSize - 1] != 0xBB) {
    return false;
  }
  const uint16_t expectedCrc =
      static_cast<uint16_t>(packet_[27]) |
      (static_cast<uint16_t>(packet_[28]) << 8);
  if (crc16Modbus(&packet_[1], 26) != expectedCrc) {
    return false;
  }

  Result parsed;
  parsed.sequence = static_cast<uint16_t>(packet_[2]) |
                    (static_cast<uint16_t>(packet_[3]) << 8);
  parsed.flags = packet_[4];
  parsed.boardQuality = packet_[5];
  for (uint8_t index = 0; index < 3; ++index) {
    const uint8_t offset = static_cast<uint8_t>(6 + index * 7);
    parsed.targets[index].number = packet_[offset];
    parsed.targets[index].source = static_cast<TargetSource>(packet_[offset + 1]);
    parsed.targets[index].x = readI16(&packet_[offset + 2]);
    parsed.targets[index].y = readI16(&packet_[offset + 4]);
    parsed.targets[index].confidence = packet_[offset + 6];
    if (parsed.targets[index].number != index + 1) {
      return false;
    }
  }
  parsed.receivedAtMs = millis();
  result_ = parsed;
  hasResult_ = true;
  return true;
}

uint16_t ThreeCircleVision::crc16Modbus(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001U)
                       : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

int16_t ThreeCircleVision::readI16(const uint8_t *data) {
  const uint16_t value = static_cast<uint16_t>(data[0]) |
                         (static_cast<uint16_t>(data[1]) << 8);
  return static_cast<int16_t>(value);
}

}  // namespace ThreeCircleFeature
