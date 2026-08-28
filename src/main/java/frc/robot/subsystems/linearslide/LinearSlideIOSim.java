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

public class LinearSlideIOSim extends LinearSlideIOTalonFX {
  // private final TalonFXSimState slideSimState;
  private TalonFXSimState motorSim;

  private final ElevatorSim slideSimModel =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          LinearSlideConstants.LinearSlideSim.slideSimGearing,
          LinearSlideConstants.LinearSlideSim.jkGMetersSquared,
          LinearSlideConstants.LinearSlideSim.linearSlideDrumRadius.in(Meters),
          LinearSlideConstants.LinearSlideSim.linearSlideMinLength.in(Meters),
          LinearSlideConstants.LinearSlideSim.linearSlideMaxLength.in(Meters),
          true,
          LinearSlideConstants.LinearSlideSim.startingHeight.in(Meters));

  public LinearSlideIOSim() {
    super();
    this.motorSim = super.getMotor().getSimState();
    // slideSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(LinearSlideIOInputs inputs) {
    motorSim = super.getMotor().getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    slideSimModel.setInputVoltage(motorSim.getMotorVoltage());
    slideSimModel.update(TimedRobot.kDefaultPeriod);

    motorSim.setRawRotorPosition(
        slideSimModel.getPositionMeters() * LinearSlideConstants.LinearSlideSim.slideSimGearing);
    motorSim.setRotorVelocity(
        slideSimModel.getVelocityMetersPerSecond()
            * LinearSlideConstants.LinearSlideSim.slideSimGearing);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(slideSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);

   SimMechs.getInstance().updateLinearSlide(Meters.of(slideSimModel.getPositionMeters()));
  }
}
