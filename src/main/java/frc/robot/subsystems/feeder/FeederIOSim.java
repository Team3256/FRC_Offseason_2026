// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

// TODO: FILL

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

public class FeederIOSim extends FeederIOTalonFX {
  private FlywheelSim feederSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              FeederConstants.kUseFOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2),
              FeederConstants.SimulationConstants.rollerMomentOfInertia,
              FeederConstants.SimulationConstants.rollerGearingRatio),
          FeederConstants.kUseFOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2));

  private final TalonFXSimState motorLeftSim;
  private final TalonFXSimState motorRightSim;

  public FeederIOSim() {
    super();
    motorLeftSim = super.getFeederMotorLeft().getSimState();
    motorRightSim = super.getFeederMotorRight().getSimState();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {

    // Update battery voltage
    motorLeftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorRightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update physics models
    feederSimModel.setInput(motorLeftSim.getMotorVoltage());
    feederSimModel.update(LoggedRobot.defaultPeriodSecs);

    double motorRPS = feederSimModel.getAngularVelocityRPM() / 60;
    motorLeftSim.setRotorVelocity(motorRPS);
    motorLeftSim.addRotorPosition(motorRPS * LoggedRobot.defaultPeriodSecs);
    motorRightSim.setRotorVelocity(motorRPS);
    motorRightSim.addRotorPosition(motorRPS * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(feederSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
    SimMechs.getInstance()
        .updateFeeder(
            Degrees.of(
                Math.toDegrees(motorRPS)
                    * LoggedRobot.defaultPeriodSecs
                    * FeederConstants.SimulationConstants.kAngularVelocityScalar));
  }
}
