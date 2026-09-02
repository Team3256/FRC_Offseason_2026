// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederMotorLeftVoltage = 0.0;
    public double feederMotorLeftVelocity = 0.0;
    public double feederMotorLeftStatorCurrent = 0.0;
    public double feederMotorLeftSupplyCurrent = 0.0;
    public double feederMotorLeftTemperature = 0.0;

    public double feederMotorRightVoltage = 0.0;
    public double feederMotorRightVelocity = 0.0;
    public double feederMotorRightStatorCurrent = 0.0;
    public double feederMotorRightSupplyCurrent = 0.0;
    public double feederMotorRightTemperature = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getFeederMotorLeft() {
    return new TalonFX(0);
  }

  public default TalonFX getFeederMotorRight() {
    return new TalonFX(0);
  }

  public default void off() {}
}
