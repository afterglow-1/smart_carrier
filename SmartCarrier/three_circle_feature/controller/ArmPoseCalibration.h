#pragma once

#include <Arduino.h>

#include "ThreeCircleVision.h"

namespace ThreeCircleFeature {

enum class WorkZone : uint8_t {
  RoughProcessing = 0,
  TemporaryStorage = 1,
};

struct ArmPose {
  // These fields map to the mechanisms already present in SmartCarrier:
  // M5 base step/dir axis, ID6 boom axis, ID7 lift axis and servo ID4 gripper.
  int32_t basePulses;
  float boomPosition;
  float liftPosition;
  float gripperAngleDeg;
  bool calibrated;
};

struct PixelCorrection {
  int16_t referenceX;
  int16_t referenceY;
  float basePulsesPerPixelX;
  float basePulsesPerPixelY;
  float boomPerPixelX;
  float boomPerPixelY;
  float liftPerPixelX;
  float liftPerPixelY;
  bool calibrated;
};

struct StationCalibration {
  ArmPose safePose;
  ArmPose trayPickupPose;
  ArmPose numberedTargetPoses[3];
  ArmPose trayReturnPose;
  PixelCorrection corrections[3];
};

// Index order: [zone][batch - 1]. Every entry is deliberately uncalibrated.
// Populate these values during raised-vehicle commissioning before enabling
// arm motion in the feature main program.
constexpr StationCalibration kStationCalibration[2][2] = {};

inline const StationCalibration &stationCalibration(
    WorkZone zone, uint8_t batch) {
  const uint8_t zoneIndex = static_cast<uint8_t>(zone);
  const uint8_t batchIndex = (batch <= 1) ? 0 : 1;
  return kStationCalibration[zoneIndex][batchIndex];
}

inline bool correctedTargetPose(
    WorkZone zone,
    uint8_t batch,
    const Target &target,
    ArmPose &output) {
  if (target.number < 1 || target.number > 3) {
    return false;
  }
  const StationCalibration &station = stationCalibration(zone, batch);
  const uint8_t index = target.number - 1;
  const ArmPose &nominal = station.numberedTargetPoses[index];
  const PixelCorrection &correction = station.corrections[index];
  if (!nominal.calibrated || !correction.calibrated) {
    return false;
  }

  const float dx = static_cast<float>(target.x - correction.referenceX);
  const float dy = static_cast<float>(target.y - correction.referenceY);
  output = nominal;
  output.basePulses += lroundf(
      correction.basePulsesPerPixelX * dx +
      correction.basePulsesPerPixelY * dy);
  output.boomPosition += correction.boomPerPixelX * dx +
                         correction.boomPerPixelY * dy;
  output.liftPosition += correction.liftPerPixelX * dx +
                         correction.liftPerPixelY * dy;
  return true;
}

inline bool stationFullyCalibrated(WorkZone zone, uint8_t batch) {
  const StationCalibration &station = stationCalibration(zone, batch);
  if (!station.safePose.calibrated || !station.trayPickupPose.calibrated ||
      !station.trayReturnPose.calibrated) {
    return false;
  }
  for (uint8_t index = 0; index < 3; ++index) {
    if (!station.numberedTargetPoses[index].calibrated ||
        !station.corrections[index].calibrated) {
      return false;
    }
  }
  return true;
}

}  // namespace ThreeCircleFeature
