// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public final class SimMechs {

  public final Mechanism2d mech =
      new Mechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1.0);

  private final MechanismRoot2d shooterRoot =
      mech.getRoot(
          "Shooter",
          Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2 - 0.05,
          Inches.of(14).in(Meters));

  private final MechanismLigament2d shooterWheelViz =
      shooterRoot.append(
          new MechanismLigament2d(
              "Shooter Wheel", Inches.of(4).in(Meters), 0.0, 4, new Color8Bit(Color.kPink)));
  private final MechanismLigament2d shooterWheelFollowerViz =
      shooterRoot.append(
          new MechanismLigament2d(
              "Shooter Wheel Follower",
              Inches.of(6).in(Meters),
              180.0,
              4,
              new Color8Bit(Color.kDeepPink)));

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

  public void updateShooterWheel(Angle delta, Angle delta_F) {
    shooterWheelViz.setAngle(shooterWheelViz.getAngle() + delta.in(Degrees));
    shooterWheelFollowerViz.setAngle(shooterWheelFollowerViz.getAngle() + delta_F.in(Degrees));
  }

  public void updateClimb(Angle of) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateClimb'");
  }
}
