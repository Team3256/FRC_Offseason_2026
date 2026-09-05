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
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class LinearSlideIOTalonFX implements LinearSlideIO {

  // private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final DifferentialPositionVoltage positionRequest =
      new DifferentialPositionVoltage(0.0, LinearSlideConstants.differenceTarget)
          .withAverageSlot(0)
          .withDifferentialSlot(2)
          .withEnableFOC(LinearSlideConstants.kUseFOC);
  // apparently target slot doesn't exist but it does in c++
  private final TalonFX leftSlideMotor = new TalonFX(LinearSlideConstants.leftMotorID);
  private final TalonFX rightSlideMotor = new TalonFX(LinearSlideConstants.rightMotorID);

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

  private final SimpleDifferentialMechanism<TalonFX> differentialMechanism =
      new SimpleDifferentialMechanism<TalonFX>(
          TalonFX::new, LinearSlideConstants.differentialConstants);

  // private final MotionMagicVoltage motionMagicRequest =
  // new MotionMagicVoltage(0).withSlot(0).withEnableFOC(LinearSlideConstants.kUseFOC);
  private final DifferentialMotionMagicVoltage motionMagicRequest =
      new DifferentialMotionMagicVoltage(0.0, LinearSlideConstants.differenceTarget)
          .withAverageSlot(0)
          .withDifferentialSlot(2)
          .withEnableFOC(LinearSlideConstants.kUseFOC);

  private final DifferentialVoltage voltageRequest =
      new DifferentialVoltage(0.0, LinearSlideConstants.differenceTarget)
          .withDifferentialSlot(2)
          .withEnableFOC(LinearSlideConstants.kUseFOC);

  public LinearSlideIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        rightSlideMotor,
        LinearSlideConstants.rightMotorConfigs,
        LinearSlideConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(
        leftSlideMotor,
        LinearSlideConstants.leftMotorConfigs,
        LinearSlideConstants.flashConfigRetries);

    // apply configs is private in differentialmechanisms class???

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
    differentialMechanism.setControl(
        motionMagicRequest
            // erroring .withTargetPosition(target) 
            .withAverageSlot(0)
            .withDifferentialPosition(LinearSlideConstants.differenceTarget));
    differentialMechanism.setControl(
        positionRequest
            .withAverageSlot(0)
            .withDifferentialPosition(LinearSlideConstants.differenceTarget));
  }

  @Override
  public void setExtendedPosition(double target) {
    if (LinearSlideConstants.kUseMotionMagic) {
      differentialMechanism.setControl(
          motionMagicRequest
              .withAverageSlot(1)
              .withDifferentialPosition(LinearSlideConstants.differenceTarget));
    } else {
      differentialMechanism.setControl(
          positionRequest
              .withAverageSlot(1)
              .withDifferentialPosition(LinearSlideConstants.differenceTarget));
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
    differentialMechanism.setControl(
        voltageRequest
            .withAverageOutput(volts)
            .withDifferentialPosition(LinearSlideConstants.differenceTarget));
  }

  /* */

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
