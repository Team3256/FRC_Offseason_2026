// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  public final boolean[] motorOpposed = {
    false, false, true, true
  }; // set what motors are opposed to leader motor

  public final int NUM_MOTORS = ShooterConstants.shooters.length;

  @AutoLog
  public static class ShooterIOInputs {
    public double[] shooterMotorVoltages = new double[NUM_MOTORS];
    public double[] shooterMotorVelocitys = new double[NUM_MOTORS];
    public double[] shooterMotorStatorCurrents = new double[NUM_MOTORS];
    public double[] shooterMotorSupplyCurrents = new double[NUM_MOTORS];
    public double[] shooterMotorTemperatures = new double[NUM_MOTORS];
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVoltage(double voltage) {}

  public default void setShooterVelocity(double velocity) {}

  public default TalonFX[] getMotors() {
    return new TalonFX[0];
  }

  public default void off() {}
}
