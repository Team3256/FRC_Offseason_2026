// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linearslide;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
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
  private final TalonFXSimState rightMotorSim;
  private final TalonFXSimState leftMotorSim;

  private final ElevatorSim slideSimModel =
      new ElevatorSim(
          DCMotor.getKrakenX60(2),
          LinearSlideConstants.LinearSlideSim.slideSimGearing,
          LinearSlideConstants.LinearSlideSim.carriageMass.in(Kilograms),
          LinearSlideConstants.LinearSlideSim.linearSlideDrumRadius.in(Meters),
          LinearSlideConstants.LinearSlideSim.linearSlideMinLength.in(Meters),
          LinearSlideConstants.LinearSlideSim.linearSlideMaxLength.in(Meters),
          false,
          LinearSlideConstants.LinearSlideSim.startingHeight.in(Meters));

  public LinearSlideIOSim() {
    super();
    this.rightMotorSim = super.getRightMotor().getSimState();
    this.leftMotorSim = super.getLeftMotor().getSimState();
    rightMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    leftMotorSim.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(LinearSlideIOInputs inputs) {
    rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    slideSimModel.setInputVoltage(
        (rightMotorSim.getMotorVoltage() + leftMotorSim.getMotorVoltage()) / 2.0);
    slideSimModel.update(TimedRobot.kDefaultPeriod);

    double rotorRotationsPerMeter =
        LinearSlideConstants.LinearSlideSim.slideSimGearing
            / (2.0
                * Math.PI
                * LinearSlideConstants.LinearSlideSim.linearSlideDrumRadius.in(Meters));
    double rotorPosition = slideSimModel.getPositionMeters() * rotorRotationsPerMeter;
    double rotorVelocity = slideSimModel.getVelocityMetersPerSecond() * rotorRotationsPerMeter;

    rightMotorSim.setRawRotorPosition(rotorPosition);
    rightMotorSim.setRotorVelocity(rotorVelocity);
    leftMotorSim.setRawRotorPosition(rotorPosition);
    leftMotorSim.setRotorVelocity(rotorVelocity);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(slideSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
    Logger.recordOutput("/LinearSlideSim/positionMeters", slideSimModel.getPositionMeters());
    Logger.recordOutput(
        "/LinearSlideSim/velocityMetersPerSecond", slideSimModel.getVelocityMetersPerSecond());
    SimMechs.getInstance().updateLinearSlide(Meters.of(slideSimModel.getPositionMeters()));
  }
}
