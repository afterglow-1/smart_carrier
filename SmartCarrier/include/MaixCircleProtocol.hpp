#pragma once

#include <stdint.h>

namespace smartcarrier {

// 与examples/CircleRecognize.py保持一致：第二字节为保留位0x00。
constexpr uint8_t MAIX_SET_CIRCLE_MODE[4] = {0xAA, 0x00, 0xF0, 0xBB};
constexpr uint8_t MAIX_SET_COLOR_MODE[4] = {0xAA, 0x00, 0xF1, 0xBB};
constexpr uint8_t MAIX_CIRCLE_REQUEST[4] = {0xAA, 0x00, 0x10, 0xBB};
constexpr uint8_t MAIX_CIRCLE_RESPONSE_TYPE = 0x10;
constexpr uint8_t MAIX_CIRCLE_DETECTED_FLAG = 0x01;
constexpr uint8_t MAIX_CIRCLE_PACKET_SIZE = 16;

struct MaixCirclePacket {
  bool detected;
  uint32_t frameId;
  uint16_t centerXpx;
  uint16_t centerYpx;
  uint16_t radiusPx;
  uint8_t confidence;
};

class MaixCirclePacketParser {
 public:
  bool push(uint8_t value, MaixCirclePacket &packet) {
    if (index_ == 0U) {
      if (value != 0xAA) {
        return false;
      }
      buffer_[index_++] = value;
      return false;
    }

    buffer_[index_++] = value;
    if (index_ < MAIX_CIRCLE_PACKET_SIZE) {
      return false;
    }

    const bool valid =
        buffer_[0] == 0xAA &&
        buffer_[1] == MAIX_CIRCLE_RESPONSE_TYPE &&
        buffer_[MAIX_CIRCLE_PACKET_SIZE - 1U] == 0xBB &&
        buffer_[14] == checksum(buffer_);

    if (valid) {
      packet.detected =
          (buffer_[2] & MAIX_CIRCLE_DETECTED_FLAG) != 0U;
      packet.frameId =
          static_cast<uint32_t>(buffer_[3]) |
          (static_cast<uint32_t>(buffer_[4]) << 8U) |
          (static_cast<uint32_t>(buffer_[5]) << 16U) |
          (static_cast<uint32_t>(buffer_[6]) << 24U);
      packet.centerXpx =
          static_cast<uint16_t>(buffer_[7]) |
          (static_cast<uint16_t>(buffer_[8]) << 8U);
      packet.centerYpx =
          static_cast<uint16_t>(buffer_[9]) |
          (static_cast<uint16_t>(buffer_[10]) << 8U);
      packet.radiusPx =
          static_cast<uint16_t>(buffer_[11]) |
          (static_cast<uint16_t>(buffer_[12]) << 8U);
      packet.confidence = buffer_[13];
    }

    // 若损坏帧的最后一个字节刚好是新帧头，则保留该字节继续同步。
    if (!valid && value == 0xAA) {
      buffer_[0] = 0xAA;
      index_ = 1U;
    } else {
      index_ = 0U;
    }
    return valid;
  }

  void reset() { index_ = 0U; }

 private:
  static uint8_t checksum(const uint8_t *buffer) {
    uint8_t result = 0U;
    for (uint8_t i = 1U; i <= 13U; ++i) {
      result ^= buffer[i];
    }
    return result;
  }

  uint8_t buffer_[MAIX_CIRCLE_PACKET_SIZE] = {};
  uint8_t index_ = 0U;
};

}  // namespace smartcarrier
