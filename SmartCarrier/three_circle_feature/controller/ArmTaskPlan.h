#pragma once

#include "ArmPoseCalibration.h"

namespace ThreeCircleFeature {

enum class ArmActionKind : uint8_t {
  MoveSafe,
  PickFromTray,
  AimAtNumber1,
  AimAtNumber2,
  AimAtNumber3,
  ReturnToTray,
  MoveSafeAfterTask,
};

struct ArmAction {
  ArmActionKind kind;
  ArmPose pose;
  uint8_t targetNumber;
};

struct ArmTaskPlan {
  static constexpr uint8_t kActionCount = 7;
  ArmAction actions[kActionCount];
};

inline bool buildArmTaskPlan(
    WorkZone zone,
    uint8_t batch,
    const Result &visionResult,
    ArmTaskPlan &plan) {
  if (!visionResult.valid() || !stationFullyCalibrated(zone, batch)) {
    return false;
  }
  const StationCalibration &station = stationCalibration(zone, batch);
  plan.actions[0] = {ArmActionKind::MoveSafe, station.safePose, 0};
  plan.actions[1] = {ArmActionKind::PickFromTray, station.trayPickupPose, 0};

  for (uint8_t index = 0; index < 3; ++index) {
    const Target &target = visionResult.targets[index];
    ArmPose corrected;
    if (target.number != index + 1 ||
        target.source == TargetSource::Missing ||
        !correctedTargetPose(zone, batch, target, corrected)) {
      return false;
    }
    plan.actions[index + 2] = {
        static_cast<ArmActionKind>(
            static_cast<uint8_t>(ArmActionKind::AimAtNumber1) + index),
        corrected,
        static_cast<uint8_t>(index + 1),
    };
  }

  plan.actions[5] = {ArmActionKind::ReturnToTray, station.trayReturnPose, 0};
  plan.actions[6] = {ArmActionKind::MoveSafeAfterTask, station.safePose, 0};
  return true;
}

}  // namespace ThreeCircleFeature
