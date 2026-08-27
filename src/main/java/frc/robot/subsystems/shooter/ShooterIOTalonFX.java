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

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMain);
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  public static class ShooterMotorStatus {
    public StatusSignal<Voltage> shooterMotorVoltage;
    public StatusSignal<AngularVelocity> shooterMotorVelocity;
    public StatusSignal<Current> shooterMotorStatorCurrent;
    public StatusSignal<Current> shooterMotorSupplyCurrent;
    public StatusSignal<Temperature> shooterMotorTemperature;

    public ShooterMotorStatus(TalonFX shooterMotor) {
      shooterMotorVoltage = shooterMotor.getMotorVoltage();
      shooterMotorVelocity = shooterMotor.getVelocity();
      shooterMotorStatorCurrent = shooterMotor.getStatorCurrent();
      shooterMotorSupplyCurrent = shooterMotor.getSupplyCurrent();
      shooterMotorTemperature = shooterMotor.getDeviceTemp();

      BaseStatusSignal.setUpdateFrequencyForAll(
          ShooterConstants.updateFrequency,
          shooterMotorVoltage,
          shooterMotorVelocity,
          shooterMotorStatorCurrent,
          shooterMotorSupplyCurrent,
          shooterMotorTemperature);
      PhoenixUtil.registerSignals(
          false,
          shooterMotorVoltage,
          shooterMotorVelocity,
          shooterMotorStatorCurrent,
          shooterMotorSupplyCurrent,
          shooterMotorTemperature);
    }
  }

  private final ShooterMotorStatus shooterMotorStatus = new ShooterMotorStatus(shooterMotor);

  TalonFX[] createFollowers() {
    TalonFX[] shooterMotorFollowers = new TalonFX[NUM_FOLLOWER_MOTORS];

    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      shooterMotorFollowers[i] = new TalonFX(ShooterConstants.shooterFollower);
    }
    return shooterMotorFollowers;
  }

  ShooterMotorFollowerIOInputs[] createFollowerIOInputses(TalonFX[] shooterMotors) {
    ShooterMotorFollowerIOInputs[] shooterFollowerMotors =
        new ShooterMotorFollowerIOInputs[NUM_FOLLOWER_MOTORS];

    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      TalonFX shooterMotorFollower = shooterMotors[i];
    }
    return shooterFollowerMotors;
  }

  ShooterMotorStatus[] createFollowerStatuses(TalonFX[] shooterMotorFollowers) {
    ShooterMotorStatus[] shooterMotorFollowerStatuses = new ShooterMotorStatus[NUM_FOLLOWER_MOTORS];
    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      TalonFX shooterMotorFollower = shooterMotorFollowers[i];
      shooterMotorFollowerStatuses[i] = new ShooterMotorStatus(shooterMotorFollower);
    }
    return shooterMotorFollowerStatuses;
  }

  TalonFX[] shooterMotorFollowers = createFollowers();
  ShooterMotorFollowerIOInputs[] shooterMotorFollowerIOInputses =
      createFollowerIOInputses(shooterMotorFollowers);
  ShooterMotorStatus[] shooterMotorFollowerStatuses = createFollowerStatuses(shooterMotorFollowers);

  public ShooterIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        shooterMotor, ShooterConstants.motorConfigs, ShooterConstants.flashConfigRetries);

    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      PhoenixUtil.applyMotorConfigs(
          shooterMotorFollowers[i],
          ShooterConstants.followerMotorConfigs,
          ShooterConstants.flashConfigRetries);
    }

    shooterMotor.optimizeBusUtilization();

    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      PhoenixUtil.applyMotorConfigs(
          shooterMotorFollowers[i],
          ShooterConstants.followerMotorConfigs,
          ShooterConstants.flashConfigRetries);

      Follower followReq =
          motorPositions[i].opposed
              ? new Follower(ShooterConstants.shooterMain, MotorAlignmentValue.Opposed)
              : new Follower(ShooterConstants.shooterMain, MotorAlignmentValue.Aligned);

      shooterMotorFollowers[i].optimizeBusUtilization();
      shooterMotorFollowers[i].setControl(followReq);
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.shooterMotorVoltage = shooterMotorStatus.shooterMotorVoltage.getValueAsDouble();
    inputs.shooterMotorVelocity = shooterMotorStatus.shooterMotorVelocity.getValueAsDouble();
    inputs.shooterMotorStatorCurrent =
        shooterMotorStatus.shooterMotorStatorCurrent.getValueAsDouble();
    inputs.shooterMotorSupplyCurrent =
        shooterMotorStatus.shooterMotorSupplyCurrent.getValueAsDouble();
    inputs.shooterMotorTemperature = shooterMotorStatus.shooterMotorTemperature.getValueAsDouble();

    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      ShooterMotorStatus shooterMotorFollowerStatus = shooterMotorFollowerStatuses[i];
      inputs.shooterMotorFollowerVoltages[i] =
          shooterMotorFollowerStatus.shooterMotorVoltage.getValueAsDouble();
      inputs.shooterMotorFollowerVelocitys[i] =
          shooterMotorFollowerStatus.shooterMotorVelocity.getValueAsDouble();
      inputs.shooterMotorFollowerStatorCurrents[i] =
          shooterMotorFollowerStatus.shooterMotorStatorCurrent.getValueAsDouble();
      inputs.shooterMotorFollowerSupplyCurrents[i] =
          shooterMotorFollowerStatus.shooterMotorSupplyCurrent.getValueAsDouble();
      inputs.shooterMotorFollowerTemperatures[i] =
          shooterMotorFollowerStatus.shooterMotorTemperature.getValueAsDouble();
    }
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      Follower followReq =
          motorPositions[i].opposed
              ? new Follower(ShooterConstants.shooterMain, MotorAlignmentValue.Opposed)
              : new Follower(ShooterConstants.shooterMain, MotorAlignmentValue.Aligned);
      shooterMotorFollowers[i].setControl(followReq);
    }
  }

  @Override
  public void setShooterVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotor.setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      shooterMotor.setControl(velocityRequest.withVelocity(velocity));
    }
    for (int i = 0; i < NUM_FOLLOWER_MOTORS; i++) {
      Follower followReq =
          motorPositions[i].opposed
              ? new Follower(ShooterConstants.shooterMain, MotorAlignmentValue.Opposed)
              : new Follower(ShooterConstants.shooterMain, MotorAlignmentValue.Aligned);
      shooterMotorFollowers[i].setControl(followReq);
    }
  }

  @Override
  public void off() {
    shooterMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return shooterMotor;
  }

  @Override
  public TalonFX[] getFollowerMotors() {
    return shooterMotorFollowers;
  }
}
