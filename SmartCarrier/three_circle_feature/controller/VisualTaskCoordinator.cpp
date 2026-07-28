#include "VisualTaskCoordinator.h"

namespace ThreeCircleFeature {

void VisualTaskCoordinator::beginAcquire() {
  vision_.clear();
  vision_.selectMode();
  vision_.requestResult();
  startedAtMs_ = millis();
  lastRequestAtMs_ = startedAtMs_;
  state_ = AcquireState::Waiting;
}

void VisualTaskCoordinator::update() {
  vision_.poll();
  if (state_ != AcquireState::Waiting) {
    return;
  }

  const uint32_t now = millis();
  if (vision_.hasFreshResult(kVisionResultMaxAgeMs) &&
      acceptable(vision_.result())) {
    acceptedResult_ = vision_.result();
    state_ = AcquireState::Ready;
    return;
  }
  if (now - startedAtMs_ >= kVisionAcquireTimeoutMs) {
    state_ = AcquireState::Failed;
    return;
  }
  if (now - lastRequestAtMs_ >= kVisionRequestIntervalMs) {
    vision_.requestResult();
    lastRequestAtMs_ = now;
  }
}

void VisualTaskCoordinator::cancel() {
  state_ = AcquireState::Idle;
  vision_.clear();
}

bool VisualTaskCoordinator::acceptable(const Result &result) const {
  if (!result.valid() || result.boardQuality < kMinimumBoardQuality) {
    return false;
  }
  for (uint8_t index = 0; index < 3; ++index) {
    const Target &target = result.targets[index];
    if (target.number != index + 1 ||
        target.source == TargetSource::Missing ||
        target.confidence < kMinimumTargetConfidence) {
      return false;
    }
  }
  return true;
}

}  // namespace ThreeCircleFeature
