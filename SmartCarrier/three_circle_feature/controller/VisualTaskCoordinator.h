#pragma once

#include <Arduino.h>

#include "ThreeCircleVision.h"

namespace ThreeCircleFeature {

constexpr uint8_t kMinimumBoardQuality = 55;
constexpr uint8_t kMinimumTargetConfidence = 45;
constexpr uint32_t kVisionResultMaxAgeMs = 500;
constexpr uint32_t kVisionAcquireTimeoutMs = 3000;
constexpr uint32_t kVisionRequestIntervalMs = 100;

enum class AcquireState : uint8_t {
  Idle,
  Waiting,
  Ready,
  Failed,
};

class VisualTaskCoordinator {
 public:
  explicit VisualTaskCoordinator(ThreeCircleVision &vision) : vision_(vision) {}

  void beginAcquire();
  void update();
  void cancel();

  AcquireState state() const { return state_; }
  const Result &acceptedResult() const { return acceptedResult_; }

 private:
  bool acceptable(const Result &result) const;

  ThreeCircleVision &vision_;
  AcquireState state_ = AcquireState::Idle;
  Result acceptedResult_;
  uint32_t startedAtMs_ = 0;
  uint32_t lastRequestAtMs_ = 0;
};

}  // namespace ThreeCircleFeature
