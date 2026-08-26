package frc.robot.subsystems.linearslide;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class LinearSlideConstants {
    public static final int slideMotorID = 0;

    public static final boolean kUseFOC = false;
    public static final boolean kUseMotionMagic = true;
    public static final int flashConfigRetries = 5;
    public static final double stowPosition = 0;
    public static final double intakePosition = 0;
    // get when tuning
    public static double updateFrequency = 50;
    public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(0)
                  .withKI(0)
                  .withKD(0)
                  .withKA(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
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
                  .withStatorCurrentLimit(0)
                  .withSupplyCurrentLimit(0)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerTime(0)
                  .withSupplyCurrentLowerLimit(0))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(85.46));

    
    public static final class LinearSlideSim {
        public static final double pivotSimGearing = 0;

        public static final Distance linearSlideLength = Inches.of(0);
        public static final Mass LinearSlideMass = Kilograms.of(1);
        public static final double jkGMetersSquared = 1;

        public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
        public static final Rotation2d maxAngle = Rotation2d.fromDegrees(180);
        public static final Rotation2d startingAngle = Rotation2d.fromDegrees(90);
    }
}
