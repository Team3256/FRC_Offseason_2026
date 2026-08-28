// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;
import java.util.Arrays;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX[] shooterMotors =
      Arrays.stream(
              ShooterConstants.shooters) // create talonfxs with canids defined in the constants
          .mapToObj(TalonFX::new)
          .toArray(TalonFX[]::new);

  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage>[] shooterMotorVoltages =
      Arrays.stream(shooterMotors).map(TalonFX::getMotorVoltage).toArray(StatusSignal[]::new);
  private final StatusSignal<AngularVelocity>[] shooterMotorVelocitys =
      Arrays.stream(shooterMotors).map(TalonFX::getVelocity).toArray(StatusSignal[]::new);
  private final StatusSignal<Current>[] shooterMotorStatorCurrents =
      Arrays.stream(shooterMotors).map(TalonFX::getStatorCurrent).toArray(StatusSignal[]::new);
  private final StatusSignal<Current>[] shooterMotorSupplyCurrents =
      Arrays.stream(shooterMotors).map(TalonFX::getSupplyCurrent).toArray(StatusSignal[]::new);
  private final StatusSignal<Temperature>[] shooterMotorTemperatures =
      Arrays.stream(shooterMotors).map(TalonFX::getDeviceTemp).toArray(StatusSignal[]::new);

  public ShooterIOTalonFX() {
    for (int i = 0; i < NUM_MOTORS; i++) {
      PhoenixUtil.applyMotorConfigs(
          shooterMotors[i], ShooterConstants.motorConfigs, ShooterConstants.flashConfigRetries);
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          ShooterConstants.updateFrequency,
          shooterMotorVoltages[i],
          shooterMotorVelocitys[i],
          shooterMotorStatorCurrents[i],
          shooterMotorSupplyCurrents[i],
          shooterMotorTemperatures[i]);
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
      PhoenixUtil.registerSignals(
          false,
          shooterMotorVoltages[i],
          shooterMotorVelocitys[i],
          shooterMotorStatorCurrents[i],
          shooterMotorSupplyCurrents[i],
          shooterMotorTemperatures[i]);
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
      shooterMotors[i].optimizeBusUtilization();
      if (i > 0) {
        Follower followReq =
            motorOpposed[i]
                ? new Follower(ShooterConstants.shooters[0], MotorAlignmentValue.Opposed)
                : new Follower(ShooterConstants.shooters[0], MotorAlignmentValue.Aligned);
        shooterMotors[i].setControl(followReq);
      }
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      inputs.shooterMotorVoltages[i] = shooterMotorVoltages[i].getValueAsDouble();
      inputs.shooterMotorVelocitys[i] = shooterMotorVelocitys[i].getValueAsDouble();
      inputs.shooterMotorStatorCurrents[i] = shooterMotorStatorCurrents[i].getValueAsDouble();
      inputs.shooterMotorSupplyCurrents[i] = shooterMotorSupplyCurrents[i].getValueAsDouble();
      inputs.shooterMotorTemperatures[i] = shooterMotorTemperatures[i].getValueAsDouble();
    }
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterMotors[0].setVoltage(voltage);
    for (int i = 1; i < NUM_MOTORS; i++) {
      Follower followReq =
          motorOpposed[i]
              ? new Follower(ShooterConstants.shooters[0], MotorAlignmentValue.Opposed)
              : new Follower(ShooterConstants.shooters[0], MotorAlignmentValue.Aligned);
      shooterMotors[i].setControl(followReq);
    }
  }

  @Override
  public void setShooterVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotors[0].setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      shooterMotors[0].setControl(velocityRequest.withVelocity(velocity));
    }
    for (int i = 1; i < NUM_MOTORS; i++) {
      Follower followReq =
          motorOpposed[i]
              ? new Follower(ShooterConstants.shooters[0], MotorAlignmentValue.Opposed)
              : new Follower(ShooterConstants.shooters[0], MotorAlignmentValue.Aligned);
      shooterMotors[i].setControl(followReq);
    }
  }

  @Override
  public void off() {
    shooterMotors[0].setControl(new NeutralOut());
  }

  @Override
  public TalonFX[] getMotors() {
    return shooterMotors;
  }
}
