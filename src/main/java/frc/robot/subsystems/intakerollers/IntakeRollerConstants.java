// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollerConstants {

  public static final int kIntakeRollerMotorIDLeft = 46;
  public static final int kIntakeRollerMotorIDRight = 47;
  // needs to be updated

  public static double updateFrequency = 50;

  public static final int flashConfigRetries = 5;

  // Rotations per second, at the motor
  public static final double kIntakeVoltage = 8.0;

  public static final double kUnjamVoltage = -4.0;

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
                  .withStatorCurrentLimit(0)
                  .withSupplyCurrentLimit(0)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(0)
                  .withSupplyCurrentLowerTime(0));

  // these aren't updated (motor configs.withCurrentLimits")

  public static final class SimulationConstants {
    public static double rollerGearingRatio = 1;
    public static double rollerMomentOfInertia = 0.1;
    // above two not updated

    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 5;
  }
}
