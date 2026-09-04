// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

// everything in here was copied from before mostly update with real values

public final class ShooterConstants {
  // field oriented control
  public static final boolean kUseFOC = true;

  public static int[] shooters = {
    0, 0, 0, 0,
  };
  // the leader is the shooters[0] and the rest are followers

  public static double updateFrequency = 50.0;

  // pid
  public static TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKA(0).withKP(0).withKI(0).withKD(0))
          // For regenerative braking
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(0)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(0)
                  .withSupplyCurrentLowerLimit(0)
                  .withSupplyCurrentLowerTime(0));
  public static TalonFXConfiguration followerMotorConfigs = motorConfigs;

  public static final InterpolatingDoubleTreeMap hubLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(0d, 0d);

          // fake data

        }
      };

  public static final InterpolatingDoubleTreeMap feedLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(0d, 0d);
        }
      };

  public static final class SimulationConstants {
    public static double kLeftGearingRatio = 0; // TODO: Update this value
    public static double kLeftMomentOfInertia = 0; // TODO: Update this value
    public static double kAngularVelocityScalar = 0;
  }

  public static int flashConfigRetries = 0;
}
