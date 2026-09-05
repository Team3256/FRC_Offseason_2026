// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linearslide;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.Logger;

public class LinearSlideIOSim extends LinearSlideIOTalonFX {
  // private final TalonFXSimState slideSimState;
  private TalonFXSimState rightMotorSim;
  private TalonFXSimState leftMotorSim;

  private final ElevatorSim slideSimModel =
      new ElevatorSim(
          DCMotor.getKrakenX60(2),
          LinearSlideConstants.LinearSlideSim.slideSimGearing,
          LinearSlideConstants.LinearSlideSim.jkGMetersSquared,
          LinearSlideConstants.LinearSlideSim.linearSlideDrumRadius.in(Meters),
          LinearSlideConstants.LinearSlideSim.linearSlideMinLength.in(Meters),
          LinearSlideConstants.LinearSlideSim.linearSlideMaxLength.in(Meters),
          true,
          LinearSlideConstants.LinearSlideSim.startingHeight.in(Meters));

  public LinearSlideIOSim() {
    super();
    this.rightMotorSim = super.getRightMotor().getSimState();
    this.leftMotorSim = super.getLeftMotor().getSimState();
    // slideSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(LinearSlideIOInputs inputs) {
    rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    slideSimModel.setInputVoltage(rightMotorSim.getMotorVoltage());
    slideSimModel.setInputVoltage(leftMotorSim.getMotorVoltage());
    slideSimModel.update(TimedRobot.kDefaultPeriod);

    rightMotorSim.setRawRotorPosition(
        slideSimModel.getPositionMeters() * LinearSlideConstants.LinearSlideSim.slideSimGearing);
    rightMotorSim.setRotorVelocity(
        slideSimModel.getVelocityMetersPerSecond()
            * LinearSlideConstants.LinearSlideSim.slideSimGearing);

    leftMotorSim.setRawRotorPosition(
        slideSimModel.getPositionMeters() * LinearSlideConstants.LinearSlideSim.slideSimGearing);
    leftMotorSim.setRotorVelocity(
        slideSimModel.getVelocityMetersPerSecond()
            * LinearSlideConstants.LinearSlideSim.slideSimGearing);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(slideSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
    Logger.recordOutput("/LinearSlideSim/positionMeters", slideSimModel.getPositionMeters());
    Logger.recordOutput(
        "/LinearSlideSim/velocityMetersPerSecond", slideSimModel.getVelocityMetersPerSecond());
    SimMechs.getInstance().updateLinearSlide(Meters.of(slideSimModel.getPositionMeters()));
  }
}
