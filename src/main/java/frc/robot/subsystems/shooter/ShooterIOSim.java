// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class ShooterIOSim extends ShooterIOTalonFX {

  private final DCMotor motor =
      ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2);

  private final LinearSystem<N1, N1, N1> flywheelSystem =
      LinearSystemId.createFlywheelSystem(
          motor,
          ShooterConstants.SimulationConstants.kLeftMomentOfInertia,
          ShooterConstants.SimulationConstants.kLeftGearingRatio);

  private final FlywheelSim flywheelSim = new FlywheelSim(flywheelSystem, motor);

  private final TalonFXSimState shooterMotorSim;
  private final TalonFXSimState[] shooterMotorFollowerSims;

  public ShooterIOSim() {
    super();
    shooterMotorSim = super.getMotor().getSimState();
    shooterMotorFollowerSims = getMotorsSims(super.getFollowerMotors());
  }

  public TalonFXSimState[] getMotorsSims(TalonFX[] motors){
    TalonFXSimState[] motorSims = new TalonFXSimState[NUM_FOLLOWER_MOTORS];
    for (int i=0; i<NUM_FOLLOWER_MOTORS; i++){
      motorSims[i] = motors[i].getSimState();
    }
    return motorSims;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Update battery voltage
    shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    for (TalonFXSimState shooterMotorFollowerSim : shooterMotorFollowerSims) shooterMotorFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Update physics models
    flywheelSim.setInputVoltage(shooterMotorSim.getMotorVoltage());
    flywheelSim.update(LoggedRobot.defaultPeriodSecs);

    double motor1Rps = flywheelSim.getAngularVelocityRPM() / 60;
    shooterMotorSim.setRotorVelocity(motor1Rps);
    shooterMotorSim.addRotorPosition(motor1Rps * LoggedRobot.defaultPeriodSecs);
    double[] motorFollowerRps = new double [3];
    for (int i=0; i<NUM_FOLLOWER_MOTORS; i++)  {
      TalonFXSimState shooterMotorFollowerSim = shooterMotorFollowerSims[i];
      motorFollowerRps[i] = flywheelSim.getAngularVelocityRPM() / 60;
      shooterMotorFollowerSim.setRotorVelocity(motorFollowerRps[i]);
      shooterMotorFollowerSim.addRotorPosition(motorFollowerRps[i] * LoggedRobot.defaultPeriodSecs);
    }

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateShooterWheel(
            Degrees.of(
                motor1Rps
                    * 360
                    * LoggedRobot.defaultPeriodSecs
                    * ShooterConstants.SimulationConstants.kAngularVelocityScalar),
            Degrees.of(
                    motorFollowerRps[2]
                            * 360
                            * LoggedRobot.defaultPeriodSecs
                            * ShooterConstants.SimulationConstants.kAngularVelocityScalar)
                );
  }
}
