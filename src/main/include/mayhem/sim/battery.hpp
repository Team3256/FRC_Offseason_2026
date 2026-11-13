#pragma once
// mayhem::sim battery: Thevenin source (open-circuit voltage + internal and wiring resistance).
// Header-only, C++17, no dependencies.
//
// Step() takes the total signed current this step (every device's SupplyCurrent(), regeneration
// negative, plus a fixed load for the roboRIO/radio/etc.). Voltage() is the battery terminal
// voltage; BusVoltage() is what to hand the devices: the terminal voltage low-passed with
// busTimeConstant (default 5 ms, standing in for wiring inductance and controller bus
// capacitance).
//
// Why filtered: devices see the bus from the previous step, and a motor whose output is clamped
// to the bus draws more supply current when the bus drops. With several such motors that
// explicit loop has a gain above 1 (dI/dV x battery resistance), so an unfiltered bus swings
// between ~12 V and ~4 V every millisecond and can trap a flywheel. The low-pass keeps the loop
// stable (stable while dt/(tau+dt) < 2/(1+gain)); SetBusTimeConstant(0) disables it.
// Defaults: a fresh FRC SLA battery (12.6 V open circuit, ~15 mOhm) with ~8 mOhm of main breaker,
// PDH and wire.
#include <algorithm>

namespace mayhem::sim {

class Battery {
 public:
  explicit Battery(double openCircuitVolts = 12.6, double internalResistanceOhms = 0.015,
                   double wiringResistanceOhms = 0.008)
      : ocv_(openCircuitVolts), rInt_(internalResistanceOhms), rWire_(wiringResistanceOhms),
        voltage_(openCircuitVolts), bus_(openCircuitVolts) {}

  /** Total current (A) drawn this step; regenerated (negative) current raises the voltage. */
  /** Total signed current (A) this step, assuming a 1 ms step; returns BusVoltage(). */
  double Step(double totalCurrentAmps) { return Step(totalCurrentAmps, 0.001); }
  /** Step with a time step, also integrating the charge drawn (amp-hours); returns BusVoltage(). */
  double Step(double totalCurrentAmps, double dt) {
    current_ = totalCurrentAmps + baseLoad_;
    voltage_ = std::max(0.0, ocv_ - current_ * (rInt_ + rWire_));
    const double alpha = busTau_ > 0 ? dt / (busTau_ + dt) : 1.0;
    bus_ += alpha * (voltage_ - bus_);
    usedAmpHours_ += current_ * dt / 3600.0;
    return bus_;
  }
  /** Battery terminal voltage after this step's current. */
  double Voltage() const { return voltage_; }
  /** Low-passed bus voltage: pass this to TalonFx::Step / SwerveSim::Step. */
  double BusVoltage() const { return bus_; }
  /** Bus filter time constant (s); 0 = unfiltered (devices see Voltage() directly). */
  void SetBusTimeConstant(double seconds) { busTau_ = std::max(0.0, seconds); }
  double BusTimeConstant() const { return busTau_; }
  double TotalCurrent() const { return current_; }
  double UsedAmpHours() const { return usedAmpHours_; }
  /** roboRIO 2 brownout: below 6.75 V by default (RobotController::SetBrownoutVoltage). */
  bool Brownout() const { return voltage_ < brownoutVolts_; }
  void SetBrownoutVoltage(double volts) { brownoutVolts_ = volts; }
  /** Constant extra load (A): roboRIO, radio, sensors, LEDs. Default 2 A. */
  void SetBaseLoad(double amps) { baseLoad_ = amps; }
  void SetOpenCircuitVoltage(double volts) { ocv_ = volts; bus_ = std::min(bus_, volts); }
  double OpenCircuitVoltage() const { return ocv_; }
  double Resistance() const { return rInt_ + rWire_; }

 private:
  double ocv_, rInt_, rWire_, voltage_, bus_, busTau_ = 0.005;
  double current_ = 0, baseLoad_ = 2, usedAmpHours_ = 0, brownoutVolts_ = 6.75;
};

}  // namespace mayhem::sim
