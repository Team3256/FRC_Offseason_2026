// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linearslide;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class LinearSlideConstants {
  public static final int rightMotorID = 3;
  public static final int leftMotorID = 4;

  public static final boolean kUseFOC = true;
  public static final boolean kUseMotionMagic = true;
  public static final int flashConfigRetries = 5;
  public static final double stowPosition = 0; // dk yet
  public static final double intakePosition = 0; // dk yet

  public static final double jitterPosition = 0; // dk yet
  public static final double jitterIntermediate = 0; // dk yet

  public static final double differenceTarget = 0.0;
  // get when tuning
  public static double updateFrequency = 50;
  public static final TalonFXConfiguration rightMotorConfigs = // leader
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.1)
                  .withKV(0.12)
                  .withKP(10)
                  .withKI(0)
                  .withKD(0)
                  .withKA(0)
                  .withKG(0.3)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withSlot1(
              new Slot1Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(0) // change
                  .withKI(0)
                  .withKD(0) // change
                  .withKA(0)
                  .withKG(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(0)
                  .withMotionMagicCruiseVelocity(0))
          .withCurrentLimits(
              // how do u get these values actually i need to learn
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(40)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerTime(.1)
                  .withSupplyCurrentLowerLimit(20))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(85.46));

  public static final TalonFXConfiguration leftMotorConfigs =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(85.46));

  public static final DifferentialMotorConstants<TalonFXConfiguration> differentialConstants =
      new DifferentialMotorConstants<TalonFXConfiguration>()
          .withLeaderId(LinearSlideConstants.rightMotorID) // or whichever side is leader
          .withFollowerId(LinearSlideConstants.leftMotorID)
          .withAlignment(MotorAlignmentValue.Aligned)
          .withSensorToDifferentialRatio(1.0) // set to actual diff gear ratio <-- idk what this
          .withLeaderInitialConfigs(rightMotorConfigs)
          .withFollowerInitialConfigs(leftMotorConfigs)
          .withFollowerUsesCommonLeaderConfigs(true);

  public static final class LinearSlideSim {
    public static final double slideSimGearing = 30;

    public static final Distance linearSlideLength = Inches.of(10);
    public static final Distance linearSlideDrumRadius = Inches.of(10);
    // elevator sim parameter so i added it but claire doesn't know soooooooo
    public static final Distance linearSlideMinLength = Inches.of(5);
    public static final Distance linearSlideMaxLength = Inches.of(20);
    public static final Mass LinearSlideMass = Kilograms.of(1);
    public static final double jkGMetersSquared = 1;
    public static final Distance startingHeight = Inches.of(12);
  }
}
