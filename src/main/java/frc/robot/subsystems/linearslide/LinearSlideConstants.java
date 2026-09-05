// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linearslide;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class LinearSlideConstants {
  public static final int rightMotorID = 39;
  public static final int leftMotorID = 40;

  public static final boolean kUseFOC = true;
  public static final boolean kUseMotionMagic = false;
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
                  .withKP(30)
                  .withKI(0)
                  .withKD(0)
                  .withKA(0)
                  .withKG(0.3)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withSlot1(
              new Slot1Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(30) // change
                  .withKI(0)
                  .withKD(0) // change
                  .withKA(0)
                  .withKG(0))
          .withSlot2(new Slot2Configs().withKP(4))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(100)
                  .withMotionMagicCruiseVelocity(10))
          .withCurrentLimits(
              // how do u get these values actually i need to learn
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(40)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerTime(.1)
                  .withSupplyCurrentLowerLimit(20))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(30));

  public static final TalonFXConfiguration leftMotorConfigs =
      new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(30));

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

    public static final Distance linearSlideDrumRadius = Inches.of(10);
    // elevator sim parameter so i added it but claire doesn't know soooooooo
    public static final Distance linearSlideMinLength = Meters.of(0);
    public static final Distance linearSlideMaxLength = Meters.of(20);
    public static final Mass carriageMass = Kilograms.of(.1);
    public static final Distance startingHeight = linearSlideMinLength;
  }
}
