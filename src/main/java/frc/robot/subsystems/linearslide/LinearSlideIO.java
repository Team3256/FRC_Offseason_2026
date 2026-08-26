package frc.robot.subsystems.linearslide;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface LinearSlideIO {
    @AutoLog
    public class LinearSlideIOInputs {

    public double slideMotorVoltage = 0.0;
    public double slideMotorVelocity = 0.0;
    public double slideMotorPosition = 0.0;
    public double slideMotorStatorCurrent = 0.0;
    public double slideMotorSupplyCurrent = 0.0;
  }

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default void updateInputs(LinearSlideIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setPosition(double position, double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void resetPosition(double angle) {}

  public default void off() {}
  // does lin slide need a zero
}
