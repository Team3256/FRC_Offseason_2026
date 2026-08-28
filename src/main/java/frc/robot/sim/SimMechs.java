// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class SimMechs {

  public final Mechanism2d mech =
      new Mechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1.0);

  private final MechanismRoot2d feederRoot =
    mech.getRoot(
        "Feeder",
          Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2,
          Inches.of(5).in(Meters));

    private final MechanismLigament2d feederViz =
      feederRoot.append(
          new MechanismLigament2d(
              "Feeder", Inches.of(2).in(Meters), 0.0, 3, new Color8Bit(Color.kMagenta)));
  private static SimMechs instance = null;

  private SimMechs() {}

  public static SimMechs getInstance() {
    if (instance == null) {
      instance = new SimMechs();
    }
    return instance;
  }

   public void updateFeeder(Angle x) {
    feederViz.setAngle(feederViz.getAngle() + x.in(Degrees));
  }


  public void publishToNT() {
    SmartDashboard.putData("RobotSim", mech);
  }

  public void updateClimb(Angle of) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateClimb'");
  }
}
