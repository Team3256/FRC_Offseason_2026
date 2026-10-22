// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakerollers;

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

public class IntakeRollersIOSim extends IntakeRollersIOTalonFX {
  private final FlywheelSim rollerSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(2),
              IntakeRollerConstants.SimulationConstants.rollerGearingRatio,
              IntakeRollerConstants.SimulationConstants.rollerMomentOfInertia),
          DCMotor.getKrakenX60(2));

  private final TalonFXSimState motorSimLeft;
  private final TalonFXSimState motorSimRight;

  public IntakeRollersIOSim() {
    super();
    motorSimLeft = super.getIntakeRollerMotor().getSimState();
    motorSimRight = super.getIntakeRollerMotorRight().getSimState();
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    // Update battery voltage for both sim states
    motorSimLeft.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorSimRight.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Drive the physics model off the leader's commanded voltage
    rollerSimModel.setInput(motorSimLeft.getMotorVoltage());
    rollerSimModel.update(LoggedRobot.defaultPeriodSecs);

    double motorRPS = rollerSimModel.getAngularVelocityRPM() / 60;

    // Leader
    motorSimLeft.setRotorVelocity(motorRPS);
    motorSimLeft.addRotorPosition(motorRPS * LoggedRobot.defaultPeriodSecs);

    // Follower — same magnitude, opposite direction (MotorAlignmentValue.Opposed)
    motorSimRight.setRotorVelocity(-motorRPS);
    motorSimRight.addRotorPosition(-motorRPS * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage after physics effects (combine current draw from both)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            rollerSimModel.getCurrentDrawAmps(), rollerSimModel.getCurrentDrawAmps()));

    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateRollers(
            Degrees.of(
                Math.toDegrees(motorRPS)
                    * LoggedRobot.defaultPeriodSecs
                    * IntakeRollerConstants.SimulationConstants.kAngularVelocityScalar));
  }
}
