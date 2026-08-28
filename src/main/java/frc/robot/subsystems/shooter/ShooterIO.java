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
    false, true, true
  }; // set what motors are opposed to leader motor

  public final int NUM_FOLLOWER_MOTORS = motorOpposed.length;

  @AutoLog
  public static class ShooterIOInputs {
    public double shooterMotorVoltage = 0.0;
    public double shooterMotorVelocity = 0.0;
    public double shooterMotorStatorCurrent = 0.0;
    public double shooterMotorSupplyCurrent = 0.0;
    public double shooterMotorTemperature = 0.0;

    public double[] shooterMotorFollowerVoltages = new double[3];
    public double[] shooterMotorFollowerVelocitys = new double[3];
    public double[] shooterMotorFollowerStatorCurrents = new double[3];
    public double[] shooterMotorFollowerSupplyCurrents = new double[3];
    public double[] shooterMotorFollowerTemperatures = new double[3];
    // ShooterMotorFollowerIOInputs[] shooterMotorFollowers = new ShooterMotorFollowerIOInputs[3];
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVoltage(double voltage) {}

  public default void setShooterVelocity(double velocity) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default TalonFX[] getFollowerMotors() {
    return new TalonFX[0];
  }

  public default void off() {}
}
