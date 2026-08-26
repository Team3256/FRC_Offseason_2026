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

    public double slideMotorVoltage = 0.0;
    public double slideMotorVelocity = 0.0;
    public double slideMotorPosition = 0.0;
    public double slideMotorStatorCurrent = 0.0;
    public double slideMotorSupplyCurrent = 0.0;
  }

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default void updateInputs(LinearSlideIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setPosition(double position, double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void resetPosition(double angle) {}

  public default void off() {}
  // does lin slide need a zero
}
