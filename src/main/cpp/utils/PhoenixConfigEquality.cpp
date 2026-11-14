// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "utils/PhoenixConfigEquality.h"

#include <cstdio>

namespace phoenix_config_equality {

namespace {
constexpr double kEpsilon = 1e-12;

constexpr bool EpsilonEquals(double a, double b, double epsilon = kEpsilon) {
  return (a - epsilon <= b) && (a + epsilon >= b);
}
template <typename U>
constexpr bool Eq(U a, U b, double epsilon = kTalonConfigEpsilon) {
  return EpsilonEquals(a.value(), b.value(), epsilon);
}
bool Report(bool val, const char* what) {
  if (kEnableLoggingIneq && !val) {
    std::printf("%s not equal\n", what);
  }
  return val;
}
}  // namespace

bool IsEqual(const cfg::TalonFXConfiguration& a, const cfg::TalonFXConfiguration& b) {
  return IsEqual(a.Slot0, b.Slot0) && IsEqual(a.Slot1, b.Slot1) && IsEqual(a.Slot2, b.Slot2) &&
         IsEqual(a.MotorOutput, b.MotorOutput) && IsEqual(a.CurrentLimits, b.CurrentLimits) &&
         IsEqual(a.Voltage, b.Voltage) && IsEqual(a.TorqueCurrent, b.TorqueCurrent) &&
         IsEqual(a.Feedback, b.Feedback) && IsEqual(a.OpenLoopRamps, b.OpenLoopRamps) &&
         IsEqual(a.ClosedLoopRamps, b.ClosedLoopRamps) &&
         IsEqual(a.HardwareLimitSwitch, b.HardwareLimitSwitch) && IsEqual(a.Audio, b.Audio) &&
         IsEqual(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch) &&
         IsEqual(a.MotionMagic, b.MotionMagic);
}

bool IsEqual(const cfg::CANrangeConfiguration& a, const cfg::CANrangeConfiguration& b) {
  return IsEqual(a.FovParams, b.FovParams) && a.FutureProofConfigs == b.FutureProofConfigs &&
         IsEqual(a.ProximityParams, b.ProximityParams) && IsEqual(a.ToFParams, b.ToFParams);
}

bool IsEqual(const cfg::CANdiConfiguration& a, const cfg::CANdiConfiguration& b) {
  return IsEqual(a.DigitalInputs, b.DigitalInputs) && IsEqual(a.PWM1, b.PWM1) &&
         IsEqual(a.PWM2, b.PWM2) && IsEqual(a.Quadrature, b.Quadrature);
}

bool IsEqual(const cfg::FovParamsConfigs& a, const cfg::FovParamsConfigs& b) {
  return Eq(a.FOVCenterX, b.FOVCenterX, kCanrangeConfigEpsilon) &&
         Eq(a.FOVCenterY, b.FOVCenterY, kCanrangeConfigEpsilon) &&
         Eq(a.FOVRangeX, b.FOVRangeX, kCanrangeConfigEpsilon) &&
         Eq(a.FOVRangeY, b.FOVRangeY, kCanrangeConfigEpsilon);
}

bool IsEqual(const cfg::PWM1Configs& a, const cfg::PWM1Configs& b) {
  return a.SensorDirection == b.SensorDirection &&
         Eq(a.AbsoluteSensorOffset, b.AbsoluteSensorOffset, kCandiConfigEpsilon) &&
         Eq(a.AbsoluteSensorDiscontinuityPoint, b.AbsoluteSensorDiscontinuityPoint,
            kCandiConfigEpsilon);
}

bool IsEqual(const cfg::PWM2Configs& a, const cfg::PWM2Configs& b) {
  return a.SensorDirection == b.SensorDirection &&
         Eq(a.AbsoluteSensorOffset, b.AbsoluteSensorOffset, kCandiConfigEpsilon) &&
         Eq(a.AbsoluteSensorDiscontinuityPoint, b.AbsoluteSensorDiscontinuityPoint,
            kCandiConfigEpsilon);
}

bool IsEqual(const cfg::ProximityParamsConfigs& a, const cfg::ProximityParamsConfigs& b) {
  return Eq(a.MinSignalStrengthForValidMeasurement, b.MinSignalStrengthForValidMeasurement,
            kCanrangeConfigEpsilon) &&
         Eq(a.ProximityHysteresis, b.ProximityHysteresis, kCanrangeConfigEpsilon) &&
         Eq(a.ProximityThreshold, b.ProximityThreshold, kCanrangeConfigEpsilon);
}

bool IsEqual(const cfg::ToFParamsConfigs& a, const cfg::ToFParamsConfigs& b) {
  return a.UpdateMode.value == b.UpdateMode.value &&
         Eq(a.UpdateFrequency, b.UpdateFrequency, kCanrangeConfigEpsilon);
}

bool IsEqual(const cfg::QuadratureConfigs& a, const cfg::QuadratureConfigs& b) {
  return a.SensorDirection == b.SensorDirection &&
         a.QuadratureEdgesPerRotation == b.QuadratureEdgesPerRotation;
}

bool IsEqual(const cfg::DigitalInputsConfigs& a, const cfg::DigitalInputsConfigs& b) {
  return a.S1CloseState.value == b.S1CloseState.value &&
         a.S2CloseState.value == b.S2CloseState.value;
}

bool IsEqual(const cfg::CANcoderConfiguration& a, const cfg::CANcoderConfiguration& b) {
  return a.FutureProofConfigs == b.FutureProofConfigs && IsEqual(a.MagnetSensor, b.MagnetSensor);
}

bool IsEqual(const cfg::MagnetSensorConfigs& a, const cfg::MagnetSensorConfigs& b) {
  return a.SensorDirection.value == b.SensorDirection.value &&
         Eq(a.MagnetOffset, b.MagnetOffset, kCancoderConfigEpsilon) &&
         Eq(a.AbsoluteSensorDiscontinuityPoint, b.AbsoluteSensorDiscontinuityPoint,
            kCancoderConfigEpsilon);
}

template <typename Slot>
static bool SlotEqual(const Slot& a, const Slot& b) {
  return Eq(a.kP, b.kP) && Eq(a.kI, b.kI) && Eq(a.kD, b.kD) && Eq(a.kV, b.kV) && Eq(a.kS, b.kS);
}

bool IsEqual(const cfg::Slot0Configs& a, const cfg::Slot0Configs& b) {
  return Report(SlotEqual(a, b), "Slot0Configs");
}
bool IsEqual(const cfg::Slot1Configs& a, const cfg::Slot1Configs& b) {
  return Report(SlotEqual(a, b), "Slot1Configs");
}
bool IsEqual(const cfg::Slot2Configs& a, const cfg::Slot2Configs& b) {
  return Report(SlotEqual(a, b), "Slot2Configs");
}

bool IsEqual(const cfg::MotorOutputConfigs& a, const cfg::MotorOutputConfigs& b) {
  bool val = a.Inverted.value == b.Inverted.value && a.NeutralMode.value == b.NeutralMode.value &&
             Eq(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband) &&
             Eq(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle) &&
             Eq(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle);
  return Report(val, "MotorOutputConfigs");
}

bool IsEqual(const cfg::CurrentLimitsConfigs& a, const cfg::CurrentLimitsConfigs& b) {
  bool val = Eq(a.StatorCurrentLimit, b.StatorCurrentLimit) &&
             Eq(a.SupplyCurrentLimit, b.SupplyCurrentLimit) &&
             a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable &&
             a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable;
  return Report(val, "CurrentLimitsConfigs");
}

bool IsEqual(const cfg::VoltageConfigs& a, const cfg::VoltageConfigs& b) {
  bool val = Eq(a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant) &&
             EpsilonEquals(a.PeakForwardVoltage.value(), b.PeakForwardVoltage.value()) &&
             EpsilonEquals(a.PeakReverseVoltage.value(), b.PeakReverseVoltage.value());
  return Report(val, "VoltageConfigs");
}

bool IsEqual(const cfg::TorqueCurrentConfigs& a, const cfg::TorqueCurrentConfigs& b) {
  bool val = Eq(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent) &&
             Eq(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent) &&
             Eq(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband);
  return Report(val, "TorqueCurrentConfigs");
}

bool IsEqual(const cfg::FeedbackConfigs& a, const cfg::FeedbackConfigs& b) {
  bool val = Eq(a.FeedbackRotorOffset, b.FeedbackRotorOffset) &&
             Eq(a.SensorToMechanismRatio, b.SensorToMechanismRatio) &&
             Eq(a.RotorToSensorRatio, b.RotorToSensorRatio) &&
             a.FeedbackSensorSource.value == b.FeedbackSensorSource.value &&
             a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID;
  if (kEnableLoggingIneq && !val) {
    std::printf("FeedbackConfigs not equal\n");
    std::printf("A-FeedbackRotorOffset: %f, B-FeedbackRotorOffset: %f\n",
                a.FeedbackRotorOffset.value(), b.FeedbackRotorOffset.value());
    std::printf("A-SensorToMechanismRatio: %f, B-SensorToMechanismRatio: %f\n",
                a.SensorToMechanismRatio.value(), b.SensorToMechanismRatio.value());
    std::printf("A-RotorToSensorRatio: %f, B-RotorToSensorRatio: %f\n",
                a.RotorToSensorRatio.value(), b.RotorToSensorRatio.value());
    std::printf("A-FeedbackSensorSource: %d, B-FeedbackSensorSource: %d\n",
                static_cast<int>(a.FeedbackSensorSource.value),
                static_cast<int>(b.FeedbackSensorSource.value));
    std::printf("A-FeedbackRemoteSensorID: %d, B-FeedbackRemoteSensorID: %d\n",
                a.FeedbackRemoteSensorID, b.FeedbackRemoteSensorID);
  }
  return val;
}

bool IsEqual(const cfg::OpenLoopRampsConfigs& a, const cfg::OpenLoopRampsConfigs& b) {
  bool val = Eq(a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod) &&
             Eq(a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod) &&
             Eq(a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod);
  return Report(val, "OpenLoopRampsConfigs");
}

bool IsEqual(const cfg::ClosedLoopRampsConfigs& a, const cfg::ClosedLoopRampsConfigs& b) {
  bool val = Eq(a.DutyCycleClosedLoopRampPeriod, b.DutyCycleClosedLoopRampPeriod) &&
             Eq(a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod) &&
             Eq(a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod);
  return Report(val, "ClosedLoopRampsConfigs");
}

bool IsEqual(const cfg::HardwareLimitSwitchConfigs& a, const cfg::HardwareLimitSwitchConfigs& b) {
  bool val = a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable &&
             a.ForwardLimitEnable == b.ForwardLimitEnable &&
             a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable &&
             a.ReverseLimitEnable == b.ReverseLimitEnable &&
             Eq(a.ForwardLimitAutosetPositionValue, b.ForwardLimitAutosetPositionValue) &&
             Eq(a.ReverseLimitAutosetPositionValue, b.ReverseLimitAutosetPositionValue) &&
             a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID &&
             a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID &&
             a.ForwardLimitSource.value == b.ForwardLimitSource.value &&
             a.ReverseLimitSource.value == b.ReverseLimitSource.value &&
             a.ForwardLimitType.value == b.ForwardLimitType.value &&
             a.ReverseLimitType.value == b.ReverseLimitType.value;
  return Report(val, "HardwareLimitSwitchConfigs");
}

bool IsEqual(const cfg::AudioConfigs& a, const cfg::AudioConfigs& b) {
  return Report(a.BeepOnBoot == b.BeepOnBoot, "AudioConfigs");
}

bool IsEqual(const cfg::SoftwareLimitSwitchConfigs& a, const cfg::SoftwareLimitSwitchConfigs& b) {
  bool val = Eq(a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold) &&
             Eq(a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold) &&
             a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable &&
             a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable;
  return Report(val, "SoftwareLimitSwitchConfigs");
}

bool IsEqual(const cfg::MotionMagicConfigs& a, const cfg::MotionMagicConfigs& b) {
  bool val = Eq(a.MotionMagicAcceleration, b.MotionMagicAcceleration) &&
             Eq(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity) &&
             Eq(a.MotionMagicJerk, b.MotionMagicJerk);
  return Report(val, "MotionMagicConfigs");
}

}  // namespace phoenix_config_equality
