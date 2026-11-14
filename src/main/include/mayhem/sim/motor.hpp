#pragma once
// mayhem::sim DC motor model. Header-only, C++17, no dependencies.
//
// Linear brushed-DC equivalent of a brushless motor, like WPILib's frc::DCMotor:
//   V = I R + w / Kv,   torque = Kt I.
// R = V / Istall, Kt = Tstall / Istall, Kv = wfree / (V - R Ifree).
// At free speed this model still draws Ifree, i.e. Kt * Ifree of torque goes into internal
// friction. FrictionTorque() returns that value; TalonFx subtracts it (smoothly signed) from
// the shaft torque it returns, so an unloaded motor settles at the published free speed.
//
// Units: SI. Speeds in rad/s at the motor shaft, torque in N*m, current in A.
#include <cmath>

namespace mayhem::sim {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kRpmToRadPerSec = 2.0 * kPi / 60.0;

struct DcMotor {
  double nominalVoltage = 12;
  double stallTorque = 0;   // N*m
  double stallCurrent = 0;  // A
  double freeCurrent = 0;   // A
  double freeSpeed = 0;     // rad/s

  /** Winding resistance (ohm). */
  double R() const { return nominalVoltage / stallCurrent; }
  /** Torque constant (N*m/A). */
  double Kt() const { return stallTorque / stallCurrent; }
  /** Velocity constant (rad/s per V). */
  double Kv() const { return freeSpeed / (nominalVoltage - R() * freeCurrent); }
  /** Winding current at a shaft speed (rad/s) with the given applied voltage. */
  double Current(double speedRadPerSec, double volts) const {
    return volts / R() - speedRadPerSec / (Kv() * R());
  }
  /** Electromagnetic torque for a winding current. */
  double Torque(double current) const { return Kt() * current; }
  /** Internal friction torque (Kt * free current), the torque lost at free speed. */
  double FrictionTorque() const { return Kt() * freeCurrent; }
  /** Back-EMF damping dT/dw at fixed voltage (N*m per rad/s, positive). */
  double Damping() const { return Kt() / (Kv() * R()); }
  /** N identical motors geared together. */
  DcMotor WithCount(int n) const {
    return {nominalVoltage, stallTorque * n, stallCurrent * n, freeCurrent * n, freeSpeed};
  }

  // Published 12 V curves. Values are the vendors' tables, as also used by WPILib's DCMotor
  // where it has them.
  // WCP Kraken X60 motor performance (docs.wcproducts.com, Kraken X60 > Motor Performance):
  // trapezoidal 6000 RPM, 7.09 N*m, 366 A, 2 A free; FOC 5800 RPM, 9.37 N*m, 483 A, 2 A.
  static DcMotor KrakenX60() { return {12, 7.09, 366, 2, 6000 * kRpmToRadPerSec}; }
  static DcMotor KrakenX60Foc() { return {12, 9.37, 483, 2, 5800 * kRpmToRadPerSec}; }
  // WCP Kraken X44 motor performance (current table): trapezoidal 7758 RPM, 4.11 N*m, 279 A,
  // 3 A free; FOC 7368 RPM, 5.01 N*m, 329 A, 3 A. (An earlier preliminary table listed
  // 7530 RPM, 4.05 N*m, 275 A.)
  static DcMotor KrakenX44() { return {12, 4.11, 279, 3, 7758 * kRpmToRadPerSec}; }
  static DcMotor KrakenX44Foc() { return {12, 5.01, 329, 3, 7368 * kRpmToRadPerSec}; }
  // CTRE Falcon 500 (CTRE / VEX published curve): trapezoidal 6380 RPM, 4.69 N*m, 257 A,
  // 1.5 A free; FOC 6080 RPM, 5.84 N*m, 304 A, 1.5 A (WPILib DCMotor::Falcon500FOC).
  static DcMotor Falcon500() { return {12, 4.69, 257, 1.5, 6380 * kRpmToRadPerSec}; }
  static DcMotor Falcon500Foc() { return {12, 5.84, 304, 1.5, 6080 * kRpmToRadPerSec}; }
  // REV NEO (REV-21-1650 datasheet / WPILib): 5676 RPM, 2.6 N*m, 105 A, 1.8 A free.
  static DcMotor Neo() { return {12, 2.6, 105, 1.8, 5676 * kRpmToRadPerSec}; }
  // REV NEO Vortex (REV-21-1652 datasheet / WPILib): 6784 RPM, 3.6 N*m, 211 A, 3.6 A free.
  static DcMotor NeoVortex() { return {12, 3.6, 211, 3.6, 6784 * kRpmToRadPerSec}; }
  // REV NEO 550 (REV-21-1651 datasheet / WPILib): 11000 RPM, 0.97 N*m, 100 A, 1.4 A free.
  static DcMotor Neo550() { return {12, 0.97, 100, 1.4, 11000 * kRpmToRadPerSec}; }
  // CTRE Minion (store page, values marked preliminary): trapezoidal 7384 RPM, 3.1 N*m,
  // ~200 A stall. CTRE lists no free current; 1.4 A is assumed (similar-size motors).
  static DcMotor Minion() { return {12, 3.1, 200, 1.4, 7384 * kRpmToRadPerSec}; }
  // CTRE Minion with FOC (Talon FXS): 7000 RPM, 3.1 N*m, ~200 A (preliminary).
  static DcMotor MinionFoc() { return {12, 3.1, 200, 1.4, 7000 * kRpmToRadPerSec}; }
};

}  // namespace mayhem::sim
