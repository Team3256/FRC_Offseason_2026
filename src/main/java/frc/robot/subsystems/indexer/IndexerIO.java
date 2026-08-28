// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerVoltageLeft = 0.0;
    public double indexerVelocityLeft = 0.0;
    public double indexerStatorCurrentLeft = 0.0;
    public double indexerSupplyCurrentLeft = 0.0;
    public double indexerTemperatureLeft = 0.0;

    public double indexerVoltageRight = 0.0;
    public double indexerVelocityRight = 0.0;
    public double indexerSupplyCurrentRight = 0.0;
    public double indexerStatorCurrentRight = 0.0;
    public double indexerTemperatureRight = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default TalonFX getLeftMotor() {
    return new TalonFX(0);
  }

  public default TalonFX getRightMotor() {
    return new TalonFX(0);
  }

  public default TalonFX getFollowerMotor() {
    return new TalonFX(0);
  }

  public default void off() {}
}
