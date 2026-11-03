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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class LinearSlideIOTalonFX implements LinearSlideIO {

  private final TalonFX rightSlideMotor = new TalonFX(LinearSlideConstants.rightMotorID);
  private final TalonFX leftSlideMotor = new TalonFX(LinearSlideConstants.leftMotorID);

  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(LinearSlideConstants.kUseFOC);

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(LinearSlideConstants.kUseFOC);

  private final VoltageOut voltageRequest =
      new VoltageOut(0).withEnableFOC(LinearSlideConstants.kUseFOC);

  private final StatusSignal<Voltage> leftMotorVoltage = leftSlideMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> leftMotorVelocity = leftSlideMotor.getVelocity();
  private final StatusSignal<Angle> leftMotorPosition = leftSlideMotor.getPosition();
  private final StatusSignal<Current> leftMotorStatorCurrent = leftSlideMotor.getStatorCurrent();
  private final StatusSignal<Current> leftMotorSupplyCurrent = leftSlideMotor.getSupplyCurrent();

  private final StatusSignal<Voltage> rightMotorVoltage = rightSlideMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> rightMotorVelocity = rightSlideMotor.getVelocity();
  private final StatusSignal<Angle> rightMotorPosition = rightSlideMotor.getPosition();
  private final StatusSignal<Current> rightMotorStatorCurrent = rightSlideMotor.getStatorCurrent();
  private final StatusSignal<Current> rightMotorSupplyCurrent = rightSlideMotor.getSupplyCurrent();

  public LinearSlideIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        rightSlideMotor,
        LinearSlideConstants.rightMotorConfigs,
        LinearSlideConstants.flashConfigRetries);
    PhoenixUtil.applyMotorConfigs(
        leftSlideMotor,
        LinearSlideConstants.leftMotorConfigs,
        LinearSlideConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        LinearSlideConstants.updateFrequency,
        rightMotorVoltage,
        rightMotorVelocity,
        rightMotorPosition,
        rightMotorStatorCurrent,
        rightMotorSupplyCurrent,
        leftMotorVoltage,
        leftMotorVelocity,
        leftMotorPosition,
        leftMotorStatorCurrent,
        leftMotorSupplyCurrent);

    PhoenixUtil.registerSignals(
        false,
        rightMotorVoltage,
        rightMotorVelocity,
        rightMotorPosition,
        rightMotorStatorCurrent,
        rightMotorSupplyCurrent,
        leftMotorVoltage,
        leftMotorVelocity,
        leftMotorPosition,
        leftMotorStatorCurrent,
        leftMotorSupplyCurrent);
  }

  @Override
  public void updateInputs(LinearSlideIOInputs inputs) {
    inputs.rightMotorVoltage = rightMotorVoltage.getValue().in(Volts);
    inputs.rightMotorVelocity = rightMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.rightMotorPosition = rightMotorPosition.getValueAsDouble();
    inputs.rightMotorStatorCurrent = rightMotorStatorCurrent.getValue().in(Amps);
    inputs.rightMotorSupplyCurrent = rightMotorSupplyCurrent.getValue().in(Amps);

    inputs.leftMotorVoltage = leftMotorVoltage.getValue().in(Volts);
    inputs.leftMotorVelocity = leftMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.leftMotorPosition = leftMotorPosition.getValueAsDouble();
    inputs.leftMotorStatorCurrent = leftMotorStatorCurrent.getValue().in(Amps);
    inputs.leftMotorSupplyCurrent = leftMotorSupplyCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(double target) {
    setPosition(target, 0);
  }

  @Override
  public void setExtendedPosition(double target) {
    setPosition(target, 1);
  }

  // each motor runs its own PID loop to the same target
  private void setPosition(double target, int slot) {
    if (LinearSlideConstants.kUseMotionMagic) {
      rightSlideMotor.setControl(motionMagicRequest.withPosition(target).withSlot(slot));
      leftSlideMotor.setControl(motionMagicRequest.withPosition(target).withSlot(slot));
    } else {
      rightSlideMotor.setControl(positionRequest.withPosition(target).withSlot(slot));
      leftSlideMotor.setControl(positionRequest.withPosition(target).withSlot(slot));
    }
  }

  @Override
  public TalonFX getRightMotor() {
    return rightSlideMotor;
  }

  @Override
  public TalonFX getLeftMotor() {
    return leftSlideMotor;
  }

  @Override
  public void setVoltage(double volts) {
    rightSlideMotor.setControl(voltageRequest.withOutput(volts));
    leftSlideMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void off() {
    rightSlideMotor.setControl(new NeutralOut());
    leftSlideMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    rightSlideMotor.setPosition(0);
    leftSlideMotor.setPosition(0);
  }

  @Override
  public void resetPosition(double angle) {
    rightSlideMotor.setPosition(angle);
    leftSlideMotor.setPosition(angle);
  }
}
