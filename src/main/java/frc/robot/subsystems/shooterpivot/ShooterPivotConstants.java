// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooterpivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ShooterPivotConstants {
  public static final int pivotMotorId = 51;

  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = false;
  public static final int flashConfigRetries = 5;

  public static double updateFrequency = 50;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(2.7)
                  .withKP(600)
                  .withKI(0)
                  .withKD(7)
                  .withKA(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(6.2)
                  .withMotionMagicCruiseVelocity(1.304)
                  .withMotionMagicExpo_kV(0.119999)
                  .withMotionMagicExpo_kA(0.1))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(40)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(20)
                  .withSupplyCurrentLowerTime(0.1))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(30));

  public static final InterpolatingDoubleTreeMap hubLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(1.62, 0.031);
          put(2.71, 0.054);
          put(3.15, 0.064);
          put(3.95, 0.069);
          put(4.92, 0.071);
        }
      };
  public static final InterpolatingDoubleTreeMap feedLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(4.38, 0.01);
          put(6.11, 0.01);
          put(8.0, 0.01);
          put(9.42, 0.01);
          put(10.8, 0.01);
          put(12.834, 0.01);
        }
      };

  public static final class PivotSim {
    public static final double pivotSimGearing = 4;

    public static final Distance shooterPivotLength = Inches.of(11.083714);
    public static final Mass shooterPivotMass = Kilograms.of(0.01);
    public static final double jkGMetersSquared = 1;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(42.152);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(0);
  }
}
