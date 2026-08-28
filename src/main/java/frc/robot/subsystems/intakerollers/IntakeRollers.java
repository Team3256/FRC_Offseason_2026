package frc.robot.subsystems.intakerollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);
  }

  public Command intake() {
    return this.run(() -> io.setVelocity(IntakeRollerConstants.kIntakeVelocity))
        .finallyDo(io::off);
  }

  public Command outtake() {
    return this.run(() -> io.setVelocity(IntakeRollerConstants.kOuttakeVelocity))
        .finallyDo(io::off);
  }

  public Command unjam() {
    return this.run(() -> io.setVelocity(IntakeRollerConstants.kUnjamVelocity))
        .finallyDo(io::off);
  }

  public Command off() {
    return this.runOnce(io::off);
  }
}
