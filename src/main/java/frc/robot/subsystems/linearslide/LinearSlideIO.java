// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linearslide;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface LinearSlideIO {
  @AutoLog
  public class LinearSlideIOInputs {

    public double rightMotorVoltage = 0.0;
    public double rightMotorVelocity = 0.0;
    public double rightMotorPosition = 0.0;
    public double rightMotorStatorCurrent = 0.0;
    public double rightMotorSupplyCurrent = 0.0;

    public double leftMotorVoltage = 0.0;
    public double leftMotorVelocity = 0.0;
    public double leftMotorPosition = 0.0;
    public double leftMotorStatorCurrent = 0.0;
    public double leftMotorSupplyCurrent = 0.0;
  }

  public default TalonFX getRightMotor() {
    return new TalonFX(0);
  }

  public default TalonFX getLeftMotor() {
    return new TalonFX(0);
  }

  public default void updateInputs(LinearSlideIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setPosition(double position, double velocity) {}

  public default void setExtendedPosition(double target) {}

  public default void setVoltage(double voltage) {}

  public default void resetPosition(double angle) {}

  public default void off() {}

  public default void zero() {
    this.getRightMotor().setPosition(0);
    this.getLeftMotor().setPosition(0);
  }
}
