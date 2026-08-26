// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public final class SimMechs {

  public final Mechanism2d mech =
      new Mechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1.0);

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
}
