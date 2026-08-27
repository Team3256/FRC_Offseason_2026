// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.utils.AutoConfig;
import frc.robot.utils.MappedXboxController;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

  public final MappedXboxController m_driverController =
      new MappedXboxController(ControllerConstants.kDriverControllerPort, "Driver");
  public final MappedXboxController m_operatorController =
      new MappedXboxController(ControllerConstants.kOperatorControllerPort, "Operator");

  /// sim file for intakepivot needs to be added -- seems like its not been merged yet

  private AutoChooser autoChooser = new AutoChooser();

  private List<AutoConfig> autos = new ArrayList<>();

  private SendableChooser<AutoConfig> autoVisualizer = new SendableChooser<AutoConfig>();
  private Field2d field2d = new Field2d();

  public RobotContainer() {

    configureSwerve();
    configureChoreoAutoChooser();
    configureOperatorBinds();
    configureAutoVisualizer();
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  private final Feeder feeder =
      new Feeder(true, Utils.isSimulation() ? new FeederIOSim() : new FeederIOTalonFX());

  private void configureOperatorBinds() {
    m_operatorController.a().onTrue(feeder.startFeeding());
  }

  private void configureChoreoAutoChooser() {}

  private void configureAutoVisualizer() {

    for (AutoConfig auto : autos) {
      autoVisualizer.addOption(auto.name, auto);
    }

    autoVisualizer.onChange(
        (trajNames) -> {
          autoChooser.select(trajNames.name);
        });

    SmartDashboard.putData("Auto Visualizer", autoVisualizer);
    SmartDashboard.putData("Field Visualize", field2d);
  }

  private void configureSwerve() {
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  }

  public void periodic() {}
}
