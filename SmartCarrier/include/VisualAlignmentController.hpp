#pragma once

#include <math.h>
#include <stdint.h>

namespace smartcarrier {

// 视觉闭环只负责决定“下一次修正动作”，不直接绑定UART或具体电机库。
// 后续MaixCAM串口解析完成后，只需调用submitCircleCenter()/submitNoDetection()；
// 主程序则通过AlignmentMotorPort把动作转交给底盘、M5和ID6。
class AlignmentMotorPort {
 public:
  virtual ~AlignmentMotorPort() {}

  // 坐标约定：forwardMm为车头方向，leftMm为车体左侧方向。
  virtual bool moveChassisRelative(float forwardMm, float leftMm) = 0;

  // 正负方向由适配层转换成当前M5和ID6的实际电机方向。
  virtual bool rotateArmBaseRelative(float degrees) = 0;
  virtual bool moveExtensionRelative(float millimeters) = 0;

  // 本控制器每次只发出一个动作，动作完成后重新采集视觉数据。
  // 允许适配层在查询过程中推进AccelStepper或轮询TTL驱动器状态。
  virtual bool isMotionBusy() = 0;
  virtual void stopAllAlignmentMotors() = 0;
};

enum class AlignmentState : uint8_t {
  IDLE,
  WAITING_FOR_SAMPLE,
  MOVING,
  SETTLING,
  ALIGNED,
  FAULT
};

enum class AlignmentFault : uint8_t {
  NONE,
  INVALID_CONFIGURATION,
  SAMPLE_TIMEOUT,
  TOO_MANY_LOST_FRAMES,
  MOTION_REJECTED,
  MOTION_TIMEOUT,
  ITERATION_LIMIT,
  CALIBRATION_COMMAND_ZERO
};

enum class AlignmentActuator : uint8_t {
  NONE,
  CHASSIS,
  ARM_BASE,
  EXTENSION
};

struct CircleObservation {
  bool detected;
  float centerXpx;
  float centerYpx;
  float radiusPx;
  float confidence;
  uint32_t frameId;
  uint32_t receivedAtMs;
};

struct VisualAlignmentConfig {
  // 当夹爪中心实际位于目标圆心时，相机应看到的圆心像素。
  // 该点必须实机示教，不应默认认为等于图像中心。
  float targetCenterXpx = 160.0F;
  float targetCenterYpx = 120.0F;

  // 可关闭底盘粗调，使全部误差仅由M5和ID6闭环修正。
  bool enableChassisCorrection = true;
  // 启用底盘粗调时，图像误差进入该范围后才使用M5和ID6精调。
  float coarseThresholdPx = 25.0F;
  float fineToleranceXpx = 3.0F;
  float fineToleranceYpx = 3.0F;
  uint8_t requiredStableFrames = 5;

  // 可选的识别质量门限。radiusPx或confidence暂不可用时可保持默认值。
  float minimumRadiusPx = 0.0F;
  float maximumRadiusPx = 10000.0F;
  float minimumConfidence = 0.0F;

  // 像素误差到车体位移的2x2标定矩阵：
  // [forwardMm]   [forwardFromX  forwardFromY] [errorXpx]
  // [leftMm   ] = [leftFromX     leftFromY   ] [errorYpx]
  //
  // 默认全零是有意设置的安全值：未标定时控制器拒绝启动。
  float chassisForwardMmPerPixelX = 0.0F;
  float chassisForwardMmPerPixelY = 0.0F;
  float chassisLeftMmPerPixelX = 0.0F;
  float chassisLeftMmPerPixelY = 0.0F;

  // 像素误差到M5相对角度、ID6相对伸缩量的局部雅可比矩阵：
  // [baseDegrees] [baseFromX       baseFromY     ] [errorXpx]
  // [extensionMm] [extensionFromX  extensionFromY] [errorYpx]
  float armBaseDegPerPixelX = 0.0F;
  float armBaseDegPerPixelY = 0.0F;
  float extensionMmPerPixelX = 0.0F;
  float extensionMmPerPixelY = 0.0F;

  // 单次修正限幅。闭环会在每次动作完成后重新拍照，避免一次修正过量。
  float maximumChassisStepMm = 40.0F;
  float minimumChassisStepMm = 1.0F;
  float maximumArmBaseStepDeg = 5.0F;
  float minimumArmBaseStepDeg = 0.2F;
  float maximumExtensionStepMm = 10.0F;
  float minimumExtensionStepMm = 0.5F;

  uint32_t sampleTimeoutMs = 800U;
  uint32_t motionTimeoutMs = 5000U;
  uint32_t settleTimeMs = 180U;
  uint8_t maximumLostFrames = 5;
  uint16_t maximumCorrectionIterations = 40;
};

class VisualAlignmentController {
 public:
  explicit VisualAlignmentController(AlignmentMotorPort &motors)
      : motors_(motors) {}

  void configure(const VisualAlignmentConfig &config) {
    config_ = config;
    configured_ = validateConfiguration(config_);
    if (!configured_ && state_ != AlignmentState::IDLE) {
      enterFault(AlignmentFault::INVALID_CONFIGURATION);
    }
  }

  bool start(uint32_t nowMs) {
    if (!configured_) {
      enterFault(AlignmentFault::INVALID_CONFIGURATION);
      return false;
    }

    state_ = AlignmentState::WAITING_FOR_SAMPLE;
    fault_ = AlignmentFault::NONE;
    lastActuator_ = AlignmentActuator::NONE;
    stableFrames_ = 0;
    lostFrames_ = 0;
    correctionIterations_ = 0;
    hasPendingObservation_ = false;
    lastConsumedFrameId_ = 0U;
    acceptSamplesAfterMs_ = nowMs;
    stateStartedAtMs_ = nowMs;
    lastErrorXpx_ = 0.0F;
    lastErrorYpx_ = 0.0F;
    lastCommandPrimary_ = 0.0F;
    lastCommandSecondary_ = 0.0F;
    return true;
  }

  void cancel() {
    motors_.stopAllAlignmentMotors();
    state_ = AlignmentState::IDLE;
    fault_ = AlignmentFault::NONE;
    lastActuator_ = AlignmentActuator::NONE;
    hasPendingObservation_ = false;
  }

  // 预留给MaixCAM串口接收器的入口。frameId可在相机协议尚未提供时传0。
  void submitCircleCenter(float centerXpx,
                          float centerYpx,
                          float radiusPx,
                          float confidence,
                          uint32_t frameId,
                          uint32_t receivedAtMs) {
    CircleObservation observation = {
        true,
        centerXpx,
        centerYpx,
        radiusPx,
        confidence,
        frameId,
        receivedAtMs};
    submitObservation(observation);
  }

  void submitNoDetection(uint32_t frameId, uint32_t receivedAtMs) {
    CircleObservation observation = {
        false, 0.0F, 0.0F, 0.0F, 0.0F, frameId, receivedAtMs};
    submitObservation(observation);
  }

  void submitObservation(const CircleObservation &observation) {
    if (state_ != AlignmentState::WAITING_FOR_SAMPLE) {
      return;
    }
    if (!timeReached(observation.receivedAtMs, acceptSamplesAfterMs_)) {
      return;
    }
    if (observation.frameId != 0U &&
        lastConsumedFrameId_ != 0U &&
        observation.frameId <= lastConsumedFrameId_) {
      return;
    }

    // 只保留最新帧，避免控制器处理动作前积压的旧画面。
    pendingObservation_ = observation;
    hasPendingObservation_ = true;
  }

  void update(uint32_t nowMs) {
    switch (state_) {
      case AlignmentState::WAITING_FOR_SAMPLE:
        updateWaitingForSample(nowMs);
        break;

      case AlignmentState::MOVING:
        updateMoving(nowMs);
        break;

      case AlignmentState::SETTLING:
        if (elapsedMs(nowMs, stateStartedAtMs_) >= config_.settleTimeMs) {
          state_ = AlignmentState::WAITING_FOR_SAMPLE;
          stateStartedAtMs_ = nowMs;
          acceptSamplesAfterMs_ = nowMs;
          hasPendingObservation_ = false;
        }
        break;

      case AlignmentState::IDLE:
      case AlignmentState::ALIGNED:
      case AlignmentState::FAULT:
        break;
    }
  }

  AlignmentState state() const { return state_; }
  AlignmentFault fault() const { return fault_; }
  AlignmentActuator lastActuator() const { return lastActuator_; }
  bool isRunning() const {
    return state_ == AlignmentState::WAITING_FOR_SAMPLE ||
           state_ == AlignmentState::MOVING ||
           state_ == AlignmentState::SETTLING;
  }
  bool isAligned() const { return state_ == AlignmentState::ALIGNED; }
  float lastErrorXpx() const { return lastErrorXpx_; }
  float lastErrorYpx() const { return lastErrorYpx_; }
  float lastCommandPrimary() const { return lastCommandPrimary_; }
  float lastCommandSecondary() const { return lastCommandSecondary_; }
  uint16_t correctionIterations() const { return correctionIterations_; }
  uint8_t stableFrames() const { return stableFrames_; }

 private:
  static float absolute(float value) {
    return value >= 0.0F ? value : -value;
  }

  static float maximum(float a, float b) {
    return a > b ? a : b;
  }

  static float clamp(float value, float low, float high) {
    if (value < low) {
      return low;
    }
    if (value > high) {
      return high;
    }
    return value;
  }

  static float applyMinimumMagnitude(float value, float minimumMagnitude) {
    if (value == 0.0F || absolute(value) >= minimumMagnitude) {
      return value;
    }
    return value > 0.0F ? minimumMagnitude : -minimumMagnitude;
  }

  static uint32_t elapsedMs(uint32_t nowMs, uint32_t startMs) {
    return static_cast<uint32_t>(nowMs - startMs);
  }

  static bool timeReached(uint32_t nowMs, uint32_t targetMs) {
    return static_cast<int32_t>(nowMs - targetMs) >= 0;
  }

  static bool finite(float value) {
    return isfinite(value);
  }

  static bool matrixHasEffect(float a, float b, float c, float d) {
    const float epsilon = 0.000001F;
    return absolute(a) > epsilon || absolute(b) > epsilon ||
           absolute(c) > epsilon || absolute(d) > epsilon;
  }

  static bool validateConfiguration(const VisualAlignmentConfig &config) {
    if (!finite(config.targetCenterXpx) ||
        !finite(config.targetCenterYpx) ||
        !finite(config.coarseThresholdPx) ||
        !finite(config.fineToleranceXpx) ||
        !finite(config.fineToleranceYpx)) {
      return false;
    }
    if (config.coarseThresholdPx <= 0.0F ||
        config.fineToleranceXpx <= 0.0F ||
        config.fineToleranceYpx <= 0.0F ||
        config.requiredStableFrames == 0U ||
        config.maximumLostFrames == 0U ||
        config.maximumCorrectionIterations == 0U) {
      return false;
    }
    if (config.coarseThresholdPx <=
        maximum(config.fineToleranceXpx, config.fineToleranceYpx)) {
      return false;
    }
    if (config.minimumRadiusPx < 0.0F ||
        config.maximumRadiusPx < config.minimumRadiusPx ||
        config.maximumChassisStepMm <= 0.0F ||
        config.minimumChassisStepMm < 0.0F ||
        config.minimumChassisStepMm > config.maximumChassisStepMm ||
        config.maximumArmBaseStepDeg <= 0.0F ||
        config.minimumArmBaseStepDeg < 0.0F ||
        config.minimumArmBaseStepDeg > config.maximumArmBaseStepDeg ||
        config.maximumExtensionStepMm <= 0.0F ||
        config.minimumExtensionStepMm < 0.0F ||
        config.minimumExtensionStepMm > config.maximumExtensionStepMm ||
        config.sampleTimeoutMs == 0U ||
        config.motionTimeoutMs == 0U) {
      return false;
    }
    if (config.enableChassisCorrection &&
        !matrixHasEffect(config.chassisForwardMmPerPixelX,
                         config.chassisForwardMmPerPixelY,
                         config.chassisLeftMmPerPixelX,
                         config.chassisLeftMmPerPixelY)) {
      return false;
    }
    return matrixHasEffect(config.armBaseDegPerPixelX,
                           config.armBaseDegPerPixelY,
                           config.extensionMmPerPixelX,
                           config.extensionMmPerPixelY);
  }

  void updateWaitingForSample(uint32_t nowMs) {
    if (hasPendingObservation_) {
      const CircleObservation observation = pendingObservation_;
      hasPendingObservation_ = false;
      consumeObservation(observation, nowMs);
      return;
    }

    if (elapsedMs(nowMs, stateStartedAtMs_) >= config_.sampleTimeoutMs) {
      enterFault(AlignmentFault::SAMPLE_TIMEOUT);
    }
  }

  void consumeObservation(const CircleObservation &observation,
                          uint32_t nowMs) {
    if (observation.frameId != 0U) {
      lastConsumedFrameId_ = observation.frameId;
    }

    const bool valid =
        observation.detected &&
        finite(observation.centerXpx) &&
        finite(observation.centerYpx) &&
        finite(observation.radiusPx) &&
        finite(observation.confidence) &&
        observation.radiusPx >= config_.minimumRadiusPx &&
        observation.radiusPx <= config_.maximumRadiusPx &&
        observation.confidence >= config_.minimumConfidence;

    if (!valid) {
      stableFrames_ = 0;
      ++lostFrames_;
      if (lostFrames_ >= config_.maximumLostFrames) {
        enterFault(AlignmentFault::TOO_MANY_LOST_FRAMES);
        return;
      }
      stateStartedAtMs_ = nowMs;
      return;
    }

    lostFrames_ = 0;
    lastErrorXpx_ = observation.centerXpx - config_.targetCenterXpx;
    lastErrorYpx_ = observation.centerYpx - config_.targetCenterYpx;

    const bool xAligned =
        absolute(lastErrorXpx_) <= config_.fineToleranceXpx;
    const bool yAligned =
        absolute(lastErrorYpx_) <= config_.fineToleranceYpx;
    if (xAligned && yAligned) {
      ++stableFrames_;
      if (stableFrames_ >= config_.requiredStableFrames) {
        state_ = AlignmentState::ALIGNED;
        lastActuator_ = AlignmentActuator::NONE;
        return;
      }
      stateStartedAtMs_ = nowMs;
      return;
    }

    stableFrames_ = 0;
    if (correctionIterations_ >= config_.maximumCorrectionIterations) {
      enterFault(AlignmentFault::ITERATION_LIMIT);
      return;
    }

    const float largestPixelError =
        maximum(absolute(lastErrorXpx_), absolute(lastErrorYpx_));
    if (config_.enableChassisCorrection &&
        largestPixelError > config_.coarseThresholdPx) {
      issueChassisCorrection(nowMs);
    } else {
      issueFineCorrection(nowMs);
    }
  }

  void issueChassisCorrection(uint32_t nowMs) {
    float forwardMm =
        config_.chassisForwardMmPerPixelX * lastErrorXpx_ +
        config_.chassisForwardMmPerPixelY * lastErrorYpx_;
    float leftMm =
        config_.chassisLeftMmPerPixelX * lastErrorXpx_ +
        config_.chassisLeftMmPerPixelY * lastErrorYpx_;

    forwardMm = clamp(forwardMm,
                      -config_.maximumChassisStepMm,
                      config_.maximumChassisStepMm);
    leftMm = clamp(leftMm,
                   -config_.maximumChassisStepMm,
                   config_.maximumChassisStepMm);
    forwardMm =
        applyMinimumMagnitude(forwardMm, config_.minimumChassisStepMm);
    leftMm = applyMinimumMagnitude(leftMm, config_.minimumChassisStepMm);

    if (forwardMm == 0.0F && leftMm == 0.0F) {
      enterFault(AlignmentFault::CALIBRATION_COMMAND_ZERO);
      return;
    }
    if (!motors_.moveChassisRelative(forwardMm, leftMm)) {
      enterFault(AlignmentFault::MOTION_REJECTED);
      return;
    }

    lastActuator_ = AlignmentActuator::CHASSIS;
    lastCommandPrimary_ = forwardMm;
    lastCommandSecondary_ = leftMm;
    beginMotion(nowMs);
  }

  void issueFineCorrection(uint32_t nowMs) {
    float baseDegrees =
        config_.armBaseDegPerPixelX * lastErrorXpx_ +
        config_.armBaseDegPerPixelY * lastErrorYpx_;
    float extensionMm =
        config_.extensionMmPerPixelX * lastErrorXpx_ +
        config_.extensionMmPerPixelY * lastErrorYpx_;

    baseDegrees = clamp(baseDegrees,
                        -config_.maximumArmBaseStepDeg,
                        config_.maximumArmBaseStepDeg);
    extensionMm = clamp(extensionMm,
                        -config_.maximumExtensionStepMm,
                        config_.maximumExtensionStepMm);

    // 每一帧只驱动一个精调轴。选择相对其单步上限更需要修正的轴，
    // 动作后重新观察，避免M5旋转造成的图像耦合被旧帧继续用于ID6。
    const float baseDemand =
        absolute(baseDegrees) / config_.maximumArmBaseStepDeg;
    const float extensionDemand =
        absolute(extensionMm) / config_.maximumExtensionStepMm;

    bool commandAccepted = false;
    if (baseDemand >= extensionDemand && baseDegrees != 0.0F) {
      baseDegrees =
          applyMinimumMagnitude(baseDegrees, config_.minimumArmBaseStepDeg);
      commandAccepted = motors_.rotateArmBaseRelative(baseDegrees);
      lastActuator_ = AlignmentActuator::ARM_BASE;
      lastCommandPrimary_ = baseDegrees;
      lastCommandSecondary_ = 0.0F;
    } else if (extensionMm != 0.0F) {
      extensionMm =
          applyMinimumMagnitude(extensionMm, config_.minimumExtensionStepMm);
      commandAccepted = motors_.moveExtensionRelative(extensionMm);
      lastActuator_ = AlignmentActuator::EXTENSION;
      lastCommandPrimary_ = extensionMm;
      lastCommandSecondary_ = 0.0F;
    } else {
      enterFault(AlignmentFault::CALIBRATION_COMMAND_ZERO);
      return;
    }

    if (!commandAccepted) {
      enterFault(AlignmentFault::MOTION_REJECTED);
      return;
    }
    beginMotion(nowMs);
  }

  void beginMotion(uint32_t nowMs) {
    ++correctionIterations_;
    state_ = AlignmentState::MOVING;
    stateStartedAtMs_ = nowMs;
    hasPendingObservation_ = false;
  }

  void updateMoving(uint32_t nowMs) {
    if (motors_.isMotionBusy()) {
      if (elapsedMs(nowMs, stateStartedAtMs_) >= config_.motionTimeoutMs) {
        motors_.stopAllAlignmentMotors();
        enterFault(AlignmentFault::MOTION_TIMEOUT);
      }
      return;
    }

    state_ = AlignmentState::SETTLING;
    stateStartedAtMs_ = nowMs;
    acceptSamplesAfterMs_ = nowMs + config_.settleTimeMs;
  }

  void enterFault(AlignmentFault fault) {
    motors_.stopAllAlignmentMotors();
    fault_ = fault;
    state_ = AlignmentState::FAULT;
    hasPendingObservation_ = false;
  }

  AlignmentMotorPort &motors_;
  VisualAlignmentConfig config_;
  CircleObservation pendingObservation_ = {};
  AlignmentState state_ = AlignmentState::IDLE;
  AlignmentFault fault_ = AlignmentFault::NONE;
  AlignmentActuator lastActuator_ = AlignmentActuator::NONE;
  bool configured_ = false;
  bool hasPendingObservation_ = false;
  uint8_t stableFrames_ = 0;
  uint8_t lostFrames_ = 0;
  uint16_t correctionIterations_ = 0;
  uint32_t stateStartedAtMs_ = 0U;
  uint32_t acceptSamplesAfterMs_ = 0U;
  uint32_t lastConsumedFrameId_ = 0U;
  float lastErrorXpx_ = 0.0F;
  float lastErrorYpx_ = 0.0F;
  float lastCommandPrimary_ = 0.0F;
  float lastCommandSecondary_ = 0.0F;
};

}  // namespace smartcarrier
