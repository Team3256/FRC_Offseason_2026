// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {

  public static final boolean KUseFOC = true;

  public static final int indexerMotorLeft = 1;
  public static final int indexerMotorRight = 2;

  public static double updateFrequency = 50;

  public static final int indexerVelocity = 50;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0.12).withKP(1).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(70)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(30)
                  .withSupplyCurrentLowerTime(0.1));

  public static int flashConfigRetries = 5;

  public static final class SimulationConstants {
    public static double indexerGearingRatio = 1.5;
    public static double indexerMomentOfInertia = 1.045254;

    public static double kAngularVelocityScalar = 5;
  }
}
