// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakerollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeatureFlags;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends DisableSubsystem {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public IntakeRollers(IntakeRollersIO io) {
    super(FeatureFlags.kIntakeRollersEnabled);
    this.io = io;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);
  }

  public Command intake() {
    return this.run(() -> io.setVelocity(IntakeRollerConstants.kIntakeVelocity)).finallyDo(io::off);
  }

  public Command outtake() {
    return this.run(() -> io.setVelocity(IntakeRollerConstants.kOuttakeVelocity))
        .finallyDo(io::off);
  }

  public Command unjam() {
    return this.run(() -> io.setVelocity(IntakeRollerConstants.kUnjamVelocity)).finallyDo(io::off);
  }

  public Command off() {
    return this.runOnce(io::off);
  }
}
