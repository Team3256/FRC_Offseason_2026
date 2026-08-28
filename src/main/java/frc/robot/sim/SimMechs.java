// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.linearslide.LinearSlideConstants;

public final class SimMechs {

  public final Mechanism2d mech =
      new Mechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1.0);

  private final MechanismRoot2d linearSlideRoot =
      mech.getRoot(
          "Linear Slide",
          Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2,
          LinearSlideConstants.LinearSlideSim.linearSlideMinLength.in(Meters));

  private final MechanismLigament2d linearSlideViz =
      linearSlideRoot.append(
          new MechanismLigament2d(
              "Linear Slide",
              LinearSlideConstants.LinearSlideSim.linearSlideMinLength
                  .plus(Inches.of(6))
                  .in(Meters),
              180));

  private static SimMechs instance = null;

  private SimMechs() {}

  public static SimMechs getInstance() {
    if (instance == null) {
      instance = new SimMechs();
    }
    return instance;
  }

  public void publishToNT() {
    SmartDashboard.putData("RobotSim", mech);
  }

  public void updateClimb(Angle of) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateClimb'");
  }

  public void updateLinearSlide(Distance height) {
    linearSlideViz.setLength(LinearSlideConstants.LinearSlideSim.linearSlideMaxLength.in(Meters));
  }
}
