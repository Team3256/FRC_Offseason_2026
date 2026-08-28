// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

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

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederMotorLeft = new TalonFX(FeederConstants.FeederMotorLeftID);
  final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> feederMotorLeftVoltage = feederMotorLeft.getMotorVoltage();
  private final StatusSignal<AngularVelocity> feederMotorLeftVelocity = feederMotorLeft.getVelocity();
  private final StatusSignal<Current> feederMotorLeftStatorCurrent = feederMotorLeft.getStatorCurrent();
  private final StatusSignal<Current> feederMotorLeftSupplyCurrent = feederMotorLeft.getSupplyCurrent();
  private final StatusSignal<Temperature> feederMotorLeftTemperature = feederMotorLeft.getDeviceTemp();

  private final TalonFX feederMotorRight = new TalonFX(FeederConstants.FeederMotorRightID);
  final Follower followReq =
      new Follower(FeederConstants.FeederMotorLeftID, MotorAlignmentValue.Aligned);

  private final StatusSignal<Voltage> feederMotorRightVoltage = feederMotorRight.getMotorVoltage();
  private final StatusSignal<AngularVelocity> feederMotorRightVelocity = feederMotorRight.getVelocity();
  private final StatusSignal<Current> feederMotorRightStatorCurrent = feederMotorRight.getStatorCurrent();
  private final StatusSignal<Current> feederMotorRightSupplyCurrent = feederMotorRight.getSupplyCurrent();
  private final StatusSignal<Temperature> feederMotorRightTemperature = feederMotorRight.getDeviceTemp();

  public FeederIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        feederMotorLeft, FeederConstants.motorConfigs, FeederConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(
        feederMotorRight, FeederConstants.motorConfigs, FeederConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        FeederConstants.updateFrequency,
        feederMotorLeftVoltage,
        feederMotorLeftVelocity,
        feederMotorLeftStatorCurrent,
        feederMotorLeftSupplyCurrent,
        feederMotorLeftTemperature,
        feederMotorRightVoltage,
        feederMotorRightVelocity,
        feederMotorRightStatorCurrent,
        feederMotorRightSupplyCurrent,
        feederMotorRightTemperature);
    PhoenixUtil.registerSignals(
        false,
        feederMotorLeftVoltage,
        feederMotorLeftVelocity,
        feederMotorLeftStatorCurrent,
        feederMotorLeftSupplyCurrent,
        feederMotorLeftTemperature,
        feederMotorRightVoltage,
        feederMotorRightVelocity,
        feederMotorRightStatorCurrent,
        feederMotorRightSupplyCurrent,
        feederMotorRightTemperature);
    feederMotorLeft.optimizeBusUtilization();
    feederMotorRight.optimizeBusUtilization();

    feederMotorRight.setControl(followReq);
  }

  public void updateInputs(FeederIOInputs inputs) {

    inputs.feederMotorLeftVoltage = feederMotorLeftVoltage.getValueAsDouble();
    inputs.feederMotorLeftVelocity = feederMotorLeftVelocity.getValueAsDouble();
    inputs.feederMotorLeftStatorCurrent = feederMotorLeftStatorCurrent.getValueAsDouble();
    inputs.feederMotorLeftSupplyCurrent = feederMotorLeftSupplyCurrent.getValueAsDouble();
    inputs.feederMotorLeftTemperature = feederMotorLeftTemperature.getValueAsDouble();

    inputs.feederMotorRightVoltage = feederMotorRightVoltage.getValueAsDouble();
    inputs.feederMotorRightVelocity = feederMotorRightVelocity.getValueAsDouble();
    inputs.feederMotorRightStatorCurrent = feederMotorRightStatorCurrent.getValueAsDouble();
    inputs.feederMotorRightSupplyCurrent = feederMotorRightSupplyCurrent.getValueAsDouble();
    inputs.feederMotorRightTemperature = feederMotorRightTemperature.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    feederMotorLeft.setVoltage(voltage);
    feederMotorRight.setControl(followReq);
  }
  
  public void setVelocity(double velocity) {
    feederMotorLeft.setControl(velReq.withVelocity(velocity));
  }

  public void off() {
    feederMotorLeft.setControl(new NeutralOut());
  }

  public TalonFX getFeederMotorLeft() {
    return feederMotorLeft;
  }
}
