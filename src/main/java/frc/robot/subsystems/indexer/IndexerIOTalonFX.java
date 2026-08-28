// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX indexerMotorLeft = new TalonFX(IndexerConstants.indexerMotorLeft);
  final VelocityVoltage indexerRequest = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> indexerVoltageLeft = indexerMotorLeft.getMotorVoltage();
  private final StatusSignal<AngularVelocity> indexerVelocityLeft = indexerMotorLeft.getVelocity();
  private final StatusSignal<Current> indexerStatorCurrentLeft =
      indexerMotorLeft.getStatorCurrent();
  private final StatusSignal<Current> indexerSupplyCurrentLeft =
      indexerMotorLeft.getSupplyCurrent();
  private final StatusSignal<Temperature> indexerTemperatureLeft = indexerMotorLeft.getDeviceTemp();

  private final TalonFX indexerMotorRight = new TalonFX(IndexerConstants.indexerMotorRight);
  final Follower followReq =
      new Follower(IndexerConstants.indexerMotorLeft, MotorAlignmentValue.Opposed);

  private final StatusSignal<Voltage> indexerVoltageRight = indexerMotorRight.getMotorVoltage();
  private final StatusSignal<AngularVelocity> indexerVelocityRight =
      indexerMotorRight.getVelocity();
  private final StatusSignal<Current> indexerStatorCurrentRight =
      indexerMotorRight.getStatorCurrent();
  private final StatusSignal<Current> indexerSupplyCurrentRight =
      indexerMotorRight.getSupplyCurrent();
  private final StatusSignal<Temperature> indexerTemperatureRight =
      indexerMotorRight.getDeviceTemp();

  public IndexerIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        indexerMotorLeft, IndexerConstants.motorConfigs, IndexerConstants.flashConfigRetries);
    PhoenixUtil.applyMotorConfigs(
        indexerMotorRight, IndexerConstants.motorConfigs, IndexerConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IndexerConstants.updateFrequency,
        indexerVoltageLeft,
        indexerVelocityLeft,
        indexerStatorCurrentLeft,
        indexerSupplyCurrentLeft,
        indexerTemperatureLeft,
        indexerVoltageRight,
        indexerVelocityRight,
        indexerSupplyCurrentRight,
        indexerStatorCurrentRight,
        indexerTemperatureRight);
    PhoenixUtil.registerSignals(
        false,
        indexerVoltageLeft,
        indexerVelocityLeft,
        indexerStatorCurrentLeft,
        indexerSupplyCurrentLeft,
        indexerTemperatureLeft,
        indexerVoltageRight,
        indexerVelocityRight,
        indexerSupplyCurrentRight,
        indexerStatorCurrentRight,
        indexerTemperatureRight);
    indexerMotorLeft.optimizeBusUtilization();
    indexerMotorRight.optimizeBusUtilization();

    indexerMotorRight.setControl(followReq);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    inputs.indexerVoltageLeft = indexerVoltageLeft.getValueAsDouble();
    inputs.indexerVelocityLeft = indexerVelocityLeft.getValueAsDouble();
    inputs.indexerStatorCurrentLeft = indexerStatorCurrentLeft.getValueAsDouble();
    inputs.indexerSupplyCurrentLeft = indexerSupplyCurrentLeft.getValueAsDouble();
    inputs.indexerTemperatureLeft = indexerTemperatureLeft.getValueAsDouble();

    inputs.indexerVoltageRight = indexerVoltageRight.getValueAsDouble();
    inputs.indexerVelocityRight = indexerVelocityRight.getValueAsDouble();
    inputs.indexerSupplyCurrentRight = indexerSupplyCurrentRight.getValueAsDouble();
    inputs.indexerStatorCurrentRight = indexerStatorCurrentRight.getValueAsDouble();
    inputs.indexerTemperatureRight = indexerTemperatureRight.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    indexerMotorLeft.setVoltage(voltage);
    indexerMotorRight.setControl(followReq);
  }

  @Override
  public void setVelocity(double velocity) {

    indexerMotorLeft.setControl(indexerRequest.withVelocity(velocity));
    indexerMotorRight.setControl(followReq);
  }

  @Override
  public void off() {
    indexerMotorLeft.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getLeftMotor() {
    return indexerMotorLeft;
  }

  public TalonFX getRightMotor() {
    return indexerMotorRight;
  }
}
