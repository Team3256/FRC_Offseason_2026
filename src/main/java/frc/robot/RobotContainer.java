
 // Copyright (c) 2025 FRC 3256
 // https://github.com/Team3256
 //
 // Use of this source code is governed by a
 // license that can be found in the LICENSE file at
 // the root directory of the project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.SweepDepotMayhemAutos;
import frc.robot.commands.SweepMayhemAutos;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollersIOSim;
import frc.robot.subsystems.intakerollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.linearslide.LinearSlide;
import frc.robot.subsystems.linearslide.LinearSlideIOSim;
import frc.robot.subsystems.linearslide.LinearSlideIOTalonFX;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.AutoConfig;
import frc.robot.utils.MappedXboxController;
import java.util.ArrayList;
import java.util.List;
import mayhemlib.ctre.CtreSwerve;

public class RobotContainer {

  // One chooser for all Mayhem autonomous routines.
  private final mayhemlib.auto.AutoChooser mayhemAutoChooser =
      new mayhemlib.auto.AutoChooser();

  private final SweepDepotMayhemAutos m_sweepDepotMayhemAutos;
  private final SweepMayhemAutos m_mayhemAutos;

  public final MappedXboxController m_driverController =
      new MappedXboxController(
          ControllerConstants.kDriverControllerPort,
          "Driver");

  public final MappedXboxController m_operatorController =
      new MappedXboxController(
          ControllerConstants.kOperatorControllerPort,
          "Operator");

  private final Telemetry logger =
      new Telemetry(
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final IntakeRollers intakeRollers =
      new IntakeRollers(
          true,
          Utils.isSimulation()
              ? new IntakeRollersIOSim()
              : new IntakeRollersIOTalonFX());

  private final Shooter shooter =
      new Shooter(
          true,
          Utils.isSimulation()
              ? new ShooterIOSim()
              : new ShooterIOTalonFX());

  private final ShooterPivot shooterPivot =
      new ShooterPivot(
          true,
          Utils.isSimulation()
              ? new ShooterPivotIOSim()
              : new ShooterPivotIOTalonFX());

  private final LinearSlide linearSlide =
      new LinearSlide(
          true,
          Utils.isSimulation()
              ? new LinearSlideIOSim()
              : new LinearSlideIOTalonFX());

  private final Feeder feeder =
      new Feeder(
          true,
          Utils.isSimulation()
              ? new FeederIOSim()
              : new FeederIOTalonFX());

  private final Indexer indexer =
      new Indexer(
          true,
          Utils.isSimulation()
              ? new IndexerIOSim()
              : new IndexerIOTalonFX());

  private final CommandSwerveDrivetrain drivetrain =
      TunerConstants.createDrivetrain();

  private static final Transform2d robotToShooterTransform =
      new Transform2d(0, 0, Rotation2d.kZero);

  private final ShotCalculator shotCalculator =
      new ShotCalculator(
          () -> drivetrain.getState().Pose,
          drivetrain::getFieldRelativeSpeeds,
          robotToShooterTransform);

  // Existing Choreo chooser and field visualizer.
  private AutoChooser autoChooser = new AutoChooser();

  private List<AutoConfig> autos = new ArrayList<>();

  private SendableChooser<AutoConfig> autoVisualizer =
      new SendableChooser<>();

  private Field2d field2d = new Field2d();

  private final Superstructure superstructure =
      new Superstructure(
          indexer,
          shooterPivot,
          shooter,
          intakeRollers,
          linearSlide,
          feeder,
          shotCalculator,
          shotCalculator.getRobotPoseSupplier(),
          () -> drivetrain.getState().Speeds);

  private final AutoRoutines m_autoRoutines;

  private final Vision vision =
      new Vision(
          drivetrain::addVisionMeasurement,
          new VisionIOPhotonVision(
              VisionConstants.frontRightCam,
              VisionConstants.robotToFrontRightCam),
          new VisionIOPhotonVision(
              VisionConstants.frontLeftCam,
              VisionConstants.robotToFrontLeftCam),
          new VisionIOPhotonVision(
              VisionConstants.backCam,
              VisionConstants.robotToBackCam));

  public RobotContainer() {

    // Existing Choreo autonomous factory.
    AutoFactory autoFactory =
        drivetrain.createAutoFactory(drivetrain::trajLogger);

    m_autoRoutines =
        new AutoRoutines(
            autoFactory,
            drivetrain,
            superstructure);

    // Mayhem factory shared by both Mayhem auto classes.
    var mayhemAutoFactory =
        CtreSwerve.autoFactory(drivetrain)
            .withTelemetry(true)
            .withUnbeach();

    m_sweepDepotMayhemAutos =
        new SweepDepotMayhemAutos(
            mayhemAutoFactory,
            superstructure);

    m_mayhemAutos =
        new SweepMayhemAutos(
            mayhemAutoFactory,
            superstructure);

    configureChoreoAutoChooser();
    configureMayhemAutoChooser();
    configureSwerve();
    configureOperatorBinds();
    configureAutoVisualizer();

    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  /** Registers every Mayhem autonomous routine in one chooser. */
  private void configureMayhemAutoChooser() {

    // Existing Depot routine: both Depot path segments.
    mayhemAutoChooser.addRoutine(
        "Depot Trench Sweep Depot",
        m_sweepDepotMayhemAutos::topTrenchSweepDepot);

    // Bottom trench options.
    mayhemAutoChooser.addRoutine(
        "Outpost Trench Sweep",
        m_mayhemAutos::bottomTrenchPart1Only);

    mayhemAutoChooser.addRoutine(
        "Outpost Trench Sweep x2",
        m_mayhemAutos::bottomTrenchPart1And2);

    // Top trench options.
    mayhemAutoChooser.addRoutine(
        "Depot Trench Sweep",
        m_mayhemAutos::topTrenchPart1Only);

    mayhemAutoChooser.addRoutine(
        "Depot Trench Sweep x2",
        m_mayhemAutos::topTrenchPart1And2);

    SmartDashboard.putData(
        "Mayhem Auto Chooser",
        mayhemAutoChooser);

    // Schedule the selected Mayhem routine during autonomous.
    RobotModeTriggers.autonomous()
        .whileTrue(
            mayhemAutoChooser.selectedCommandScheduler());
  }

  private void configureOperatorBinds() {

    // Driver A: Shoot.
    m_driverController.a()
        .onTrue(
            superstructure.setState(
                Superstructure.StructureState.SHOOT));

    // Driver B: Intake.
    m_driverController.b()
        .onTrue(
            superstructure.setState(
                Superstructure.StructureState.INTAKE));

    // Driver X: Jitter.
    m_driverController.x()
        .onTrue(
            superstructure.setState(
                Superstructure.StructureState.JITTER));

    // Driver Y: Home.
    m_driverController.y()
        .onTrue(
            superstructure.setState(
                Superstructure.StructureState.HOME));

    // Driver left trigger: Clear LinearSlide faults.
    m_driverController.leftTrigger()
        .onTrue(
            linearSlide.clearfaults());

    m_driverController
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> shooterPivot.setDriverOverride(true),
                () -> shooterPivot.setDriverOverride(false)));
  }

  /**
   * Keeps the existing Choreo chooser for visualization.
   * Autonomous scheduling is handled by the Mayhem chooser above.
   */
  private void configureChoreoAutoChooser() {

    autos =
        List.of(
            new AutoConfig(
                "Depot Trench Sweep",
                m_autoRoutines::topTrenchSweepAuto,
                List.of("topTrenchSweepBump")),

            new AutoConfig(
                "Outpost Trench Sweep",
                m_autoRoutines::bottomTrenchSweepAuto,
                List.of("bottomTrenchSweepBump")),

            new AutoConfig(
                "Depot Trench Sweep 2x",
                m_autoRoutines::topTrenchSweep2xAuto,
                List.of(
                    "topTrenchSweepBump2xPt1",
                    "topTrenchSweepBump2xPt2")));

    for (AutoConfig auto : autos) {
      autoChooser.addRoutine(
          auto.name,
          auto.routine);
    }

    SmartDashboard.putData(
        "auto chooser",
        autoChooser);
  }

  private void configureAutoVisualizer() {

    for (AutoConfig auto : autos) {
      autoVisualizer.addOption(
          auto.name,
          auto);
    }

    autoVisualizer.onChange(
        trajNames -> autoChooser.select(trajNames.name));

    SmartDashboard.putData(
        "Auto Visualizer",
        autoVisualizer);

    SmartDashboard.putData(
        "Field Visualize",
        field2d);
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
            .withRotationalRate(
                deadbandMultiplier * MaxAngularRate);

    SwerveRequest.FieldCentricFacingAngle azimuth =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(deadbandMultiplier * MaxSpeed);

    azimuth.HeadingController.enableContinuousInput(
        -Math.PI,
        Math.PI);

    azimuth.HeadingController.setPID(
        AzimuthTargets.aziKP,
        AzimuthTargets.aziKi,
        AzimuthTargets.aziKD);

    // Default drive with trench assist.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              ChassisSpeeds assisted =
                  drivetrain.applyTrenchAssist(
                      new ChassisSpeeds(
                          -(Math.signum(m_driverController.getLeftY())
                                  * Math.pow(m_driverController.getLeftY(), 2))
                              * MaxSpeed,
                          -(Math.signum(m_driverController.getLeftX())
                                  * Math.pow(m_driverController.getLeftX(), 2))
                              * MaxSpeed,
                          -m_driverController.getRightX() * MaxAngularRate));
              return drive
                  .withVelocityX(assisted.vxMetersPerSecond)
                  .withVelocityY(assisted.vyMetersPerSecond)
                  .withRotationalRate(assisted.omegaRadiansPerSecond);
            }));

    // Driver left bumper: Slow drive with trench assist.
    m_driverController
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () -> {
                  ChassisSpeeds assisted =
                      drivetrain.applyTrenchAssist(
                          new ChassisSpeeds(
                              -(Math.signum(m_driverController.getLeftY())
                                      * Math.pow(m_driverController.getLeftY(), 2))
                                  * SlowMaxSpeed,
                              -(Math.signum(m_driverController.getLeftX())
                                      * Math.pow(m_driverController.getLeftX(), 2))
                                  * SlowMaxSpeed,
                              -m_driverController.getRightX() * SlowMaxAngular));
                  return drive
                      .withVelocityX(assisted.vxMetersPerSecond)
                      .withVelocityY(assisted.vyMetersPerSecond)
                      .withRotationalRate(assisted.omegaRadiansPerSecond);
                }));

    // Driver right bumper: Super slow drive with trench assist.
    m_driverController
        .rightBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () -> {
                  ChassisSpeeds assisted =
                      drivetrain.applyTrenchAssist(
                          new ChassisSpeeds(
                              -(Math.signum(m_driverController.getLeftY())
                                      * Math.pow(m_driverController.getLeftY(), 2))
                                  * SuperSlowMaxSpeed,
                              -(Math.signum(m_driverController.getLeftX())
                                      * Math.pow(m_driverController.getLeftX(), 2))
                                  * SuperSlowMaxSpeed,
                              -m_driverController.getRightX() * SlowMaxAngular));
                  return drive
                      .withVelocityX(assisted.vxMetersPerSecond)
                      .withVelocityY(assisted.vyMetersPerSecond)
                      .withRotationalRate(assisted.omegaRadiansPerSecond);
                }));

    // Driver POV right: Seed field-centric.
    m_driverController
        .povRight()
        .onTrue(
            drivetrain.runOnce(
                drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(
        logger::telemeterize);

    // Driver A: Rotate to lookahead.
    m_driverController
        .a()
        .whileTrue(
            drivetrain.rotateToLookahead(
                shotCalculator,
                this::vxSupplier,
                this::vySupplier));
  }

  public void periodic() {

    field2d.setRobotPose(
        drivetrain.getState().Pose);

    superstructure.periodic();

    shotCalculator.periodic();
  }
}
