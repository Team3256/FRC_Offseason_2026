// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  public static class MotorPosition {
    int positionV;
    int positionH;
    boolean opposed;

    public MotorPosition(int i, int i1) {
      this.positionV = i;
      this.positionH = i1;
      this.opposed = (i1 == 1);

    }
  }

  public static MotorPosition[] motorPositions = {
          new MotorPosition(0, 1),
          new MotorPosition(1, 0),
          new MotorPosition(1, 1),
  };

  public final int NUM_FOLLOWER_MOTORS = motorPositions.length;

  @AutoLog
  public static class ShooterMotorFollowerIOInputs {
    public double shooterMotorFollowerVoltage = 0.0;
    public double shooterMotorFollowerVelocity = 0.0;
    public double shooterMotorFollowerStatorCurrent = 0.0;
    public double shooterMotorFollowerSupplyCurrent = 0.0;
    public double shooterMotorFollowerTemperature = 0.0;
  }


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
    //ShooterMotorFollowerIOInputs[] shooterMotorFollowers = new ShooterMotorFollowerIOInputs[3];
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
