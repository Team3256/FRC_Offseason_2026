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
import java.util.Arrays;
import org.littletonrobotics.junction.LoggedRobot;

public class ShooterIOSim extends ShooterIOTalonFX {

  private final DCMotor motor =
      ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(4) : DCMotor.getKrakenX60(4);

  private final LinearSystem<N1, N1, N1> flywheelSystem =
      LinearSystemId.createFlywheelSystem(
          motor,
          ShooterConstants.SimulationConstants.kLeftMomentOfInertia,
          ShooterConstants.SimulationConstants.kLeftGearingRatio);

  private final FlywheelSim flywheelSim = new FlywheelSim(flywheelSystem, motor);

  private final TalonFXSimState[] shooterMotorSims;

  public ShooterIOSim() {
    super();
    shooterMotorSims =
        Arrays.stream(super.getMotors()).map(TalonFX::getSimState).toArray(TalonFXSimState[]::new);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Update battery voltage
    for (TalonFXSimState shooterMotorSim : shooterMotorSims)
      shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Update physics models
    flywheelSim.setInputVoltage(shooterMotorSims[0].getMotorVoltage());
    flywheelSim.update(LoggedRobot.defaultPeriodSecs);

    double[] motorRps = new double[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      motorRps[i] = flywheelSim.getAngularVelocityRPM() / 60;
      shooterMotorSims[i].setRotorVelocity(motorRps[i]);
      shooterMotorSims[i].addRotorPosition(motorRps[i] * LoggedRobot.defaultPeriodSecs);
    }

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateShooterWheel(
            Degrees.of(
                motorRps[0]
                    * 360
                    * LoggedRobot.defaultPeriodSecs
                    * ShooterConstants.SimulationConstants.kAngularVelocityScalar));
  }
}
