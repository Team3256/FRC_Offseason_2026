// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollersIOSim;
import frc.robot.subsystems.intakerollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooterpivot.ShooterPivot;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooterpivot.ShooterPivotIOTalonFX;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveConstants.AzimuthTargets;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.AutoConfig;
import frc.robot.utils.MappedXboxController;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

  public final MappedXboxController m_driverController =
      new MappedXboxController(ControllerConstants.kDriverControllerPort, "Driver");
  public final MappedXboxController m_operatorController =
      new MappedXboxController(ControllerConstants.kOperatorControllerPort, "Operator");

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private final IntakeRollers intakeRollers =
      new IntakeRollers(
          true, Utils.isSimulation() ? new IntakeRollersIOSim() : new IntakeRollersIOTalonFX());

  private final Shooter shooter =
      new Shooter(true, Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOTalonFX());
  private final ShooterPivot shooterPivot =
      new ShooterPivot(
          true, Utils.isSimulation() ? new ShooterPivotIOSim() : new ShooterPivotIOTalonFX());
  private final Feeder feeder =
      new Feeder(true, Utils.isSimulation() ? new FeederIOSim() : new FeederIOTalonFX());
  private final Indexer indexer =
      new Indexer(true, Utils.isSimulation() ? new IndexerIOSim() : new IndexerIOTalonFX());

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private static final Transform2d robotToShooterTransform =
      new Transform2d(0, 0, Rotation2d.kZero);

  private final ShotCalculator shotCalculator =
      new ShotCalculator(
          () -> drivetrain.getState().Pose,
          drivetrain::getFieldRelativeSpeeds,
          robotToShooterTransform);

  /// sim file for intakepivot needs to be added -- seems like its not been merged yet

  private AutoChooser autoChooser = new AutoChooser();

  private List<AutoConfig> autos = new ArrayList<>();

  private SendableChooser<AutoConfig> autoVisualizer = new SendableChooser<AutoConfig>();
  private Field2d field2d = new Field2d();

  public RobotContainer() {

    configureChoreoAutoChooser();
    configureSwerve();
    configureOperatorBinds();
    configureAutoVisualizer();
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  private void configureOperatorBinds() {}

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

  private double vxSupplier() {
    return -(Math.signum(m_driverController.getLeftY())
            * Math.pow(m_driverController.getLeftY(), 2))
        * MaxSpeed;
  }

  private double vySupplier() {
    return -(Math.signum(m_driverController.getLeftX())
            * Math.pow(m_driverController.getLeftX(), 2))
        * MaxSpeed;
  }

  private void configureSwerve() {
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(deadbandMultiplier * MaxSpeed)
            .withRotationalRate(deadbandMultiplier * MaxAngularRate);

    SwerveRequest.FieldCentricFacingAngle azimuth =
        new SwerveRequest.FieldCentricFacingAngle().withDeadband(deadbandMultiplier * MaxSpeed);

    azimuth.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    azimuth.HeadingController.setPID(
        AzimuthTargets.aziKP, AzimuthTargets.aziKi, AzimuthTargets.aziKD);

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -(Math.signum(m_driverController.getLeftY())
                                * Math.pow(m_driverController.getLeftY(), 2))
                            * MaxSpeed)
                    .withVelocityY(
                        -(Math.signum(m_driverController.getLeftX())
                                * Math.pow(m_driverController.getLeftX(), 2))
                            * MaxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));

    m_driverController
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -(Math.signum(m_driverController.getLeftY())
                                    * Math.pow(m_driverController.getLeftY(), 2))
                                * SlowMaxSpeed)
                        .withVelocityY(
                            -(Math.signum(m_driverController.getLeftX())
                                    * Math.pow(m_driverController.getLeftX(), 2))
                                * SlowMaxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * SlowMaxAngular)));
    m_driverController
        .rightBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -(Math.signum(m_driverController.getLeftY())
                                    * Math.pow(m_driverController.getLeftY(), 2))
                                * SuperSlowMaxSpeed)
                        .withVelocityY(
                            -(Math.signum(m_driverController.getLeftX())
                                    * Math.pow(m_driverController.getLeftX(), 2))
                                * SuperSlowMaxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * SlowMaxAngular)));

    m_driverController.povRight().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController
        .a()
        .whileTrue(
            drivetrain.rotateToLookahead(shotCalculator, this::vxSupplier, this::vySupplier));
  }

  public void periodic() {
    field2d.setRobotPose(drivetrain.getState().Pose);
    shotCalculator.periodic();
  }
}
