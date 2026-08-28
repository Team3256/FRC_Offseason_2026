// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linearslide;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class LinearSlideIOTalonFX implements LinearSlideIO {
    
  private final TalonFX slideMotor = new TalonFX(LinearSlideConstants.slideMotorID);

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(LinearSlideConstants.kUseFOC);

  private final StatusSignal<Voltage> motorVoltage = slideMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = slideMotor.getVelocity();
  private final StatusSignal<Angle> position = slideMotor.getPosition();
  private final StatusSignal<Current> statorCurrent = slideMotor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = slideMotor.getSupplyCurrent();

  public LinearSlideIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        slideMotor, LinearSlideConstants.motorConfigs, LinearSlideConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        LinearSlideConstants.updateFrequency,
        motorVoltage,
        velocity,
        position,
        statorCurrent,
        supplyCurrent);

    PhoenixUtil.registerSignals(
        false, motorVoltage, velocity, position, statorCurrent, supplyCurrent);
  }
  @Override
  public void updateInputs(LinearSlideIOInputs inputs) {

    inputs.slideMotorVoltage = motorVoltage.getValue().in(Volts);
    inputs.slideMotorVelocity = velocity.getValue().in(RotationsPerSecond);

    inputs.slideMotorPosition = position.getValue().in(Rotations);

    inputs.slideMotorStatorCurrent = statorCurrent.getValue().in(Amps);
    inputs.slideMotorSupplyCurrent = supplyCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(double target) {
    if (LinearSlideConstants.kUseMotionMagic) {
      slideMotor.setControl(motionMagicRequest.withPosition(target));
    }
  }

  @Override
  public TalonFX getMotor() {
    return slideMotor;
  }

  @Override
  public void setVoltage(double volts) {
    slideMotor.setVoltage(volts);
  }

  @Override
  public void resetPosition(double angle) {
    slideMotor.setPosition(angle);
  }
}
