// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class IndexerIOSim extends IndexerIOTalonFX {
  private final FlywheelSim indexerSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              IndexerConstants.KUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
              IndexerConstants.SimulationConstants.indexerGearingRatio,
              IndexerConstants.SimulationConstants.indexerMomentOfInertia),
          IndexerConstants.KUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1));

  private final TalonFXSimState motorSim;

  public IndexerIOSim() {
    super();
    motorSim = super.getMotor().getSimState();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update physics models
    indexerSimModel.setInput(motorSim.getMotorVoltage());
    indexerSimModel.update(LoggedRobot.defaultPeriodSecs);

    double motorRPS = indexerSimModel.getAngularVelocityRPM() / 60;
    motorSim.setRotorVelocity(motorRPS);
    motorSim.addRotorPosition(motorRPS * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(indexerSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateIndexer(
            Degrees.of(
                Math.toDegrees(motorRPS)
                    * LoggedRobot.defaultPeriodSecs
                    * IndexerConstants.SimulationConstants.kAngularVelocityScalar));
  }
}
