package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollerConstants {

  public static final int kIntakeRollerMotorIDLeft = 46;
  public static final int kIntakeRollerMotorIDRight = 47;

  public static double updateFrequency = 50;

  // Rotations per second, at the motor
  public static final double kIntakeVelocity = 40.0;
  public static final double kOuttakeVelocity = -40.0;
  public static final double kUnjamVelocity = -15.0;

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

  public static final class SimulationConstants {
    public static double rollerGearingRatio = 1.0;
    public static double rollerMomentOfInertia = 1;

    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 5;
  }
}
