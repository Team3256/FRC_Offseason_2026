// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linearslide;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class LinearSlide extends DisableSubsystem {
  private final LinearSlideIO linearSlideIO;
  private final LinearSlideIOInputsAutoLogged linearSlideIOInputsAutoLogged =
      new LinearSlideIOInputsAutoLogged();

  public final Trigger reachedPosition = new Trigger(this::reachedPosition);

  private double reqPosition = 0.0;

  public LinearSlide(boolean enabled, LinearSlideIO linearSlideIO) {
    super(enabled);
    this.linearSlideIO = linearSlideIO;
    linearSlideIO.resetPosition(LinearSlideConstants.stowPosition);
  }

  @Override
  public void periodic() {
    super.periodic();
    linearSlideIO.updateInputs(linearSlideIOInputsAutoLogged);
    Logger.processInputs("LinearSlide", linearSlideIOInputsAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);
  }

  public Command setPosition(double position) {
    return setPosition(() -> position);
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          reqPosition = position.getAsDouble();
          linearSlideIO.setPosition(reqPosition);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> linearSlideIO.setVoltage(voltage));
  }

  public Command off() {
    return this.runOnce(linearSlideIO::off).withName("off");
  }

  public Command goToStow() {
    return this.setPosition(LinearSlideConstants.stowPosition);
  }

  public Command goToGroundIntake() {
    return this.setPosition(LinearSlideConstants.intakePosition);
  }

  public boolean reachedPosition() {
    return Util.epsilonEquals(linearSlideIOInputsAutoLogged.slideMotorPosition, reqPosition, 0.01);
  }
}
