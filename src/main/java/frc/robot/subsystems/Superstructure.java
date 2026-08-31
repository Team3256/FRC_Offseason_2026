// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.linearslide.LinearSlide;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpivot.ShooterPivot;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.utils.LoggedTracer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public enum StructureState {
    INTAKE,
    SHOOT,
    IDLE,
    CANCEL_ALL,
    HOME,
    REV,
    SHOOT_AND_INTAKE,
    REV_AND_INTAKE,
    JITTER,
    JITTER_AND_INTAKE,
    JITTER_AND_SHOOT,
  }

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  public Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Map<StructureState, Trigger> prevStateTriggers = new HashMap<StructureState, Trigger>();

  private final Timer stateTimer = new Timer();

  private final Indexer indexer;
  private final ShooterPivot shooterPivot;
  private final Shooter shooter;
  private final IntakeRollers intakeRollers;
  private final LinearSlide linearSlide;
  private final Feeder feeder;

  private final ShotCalculator shotCalculator;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final Trigger targetBlueHub = new Trigger(this::targetBlueHub);
  private final Trigger targetRedHub = new Trigger(this::targetRedHub);

  private final Trigger feedTopCorner =
          targetBlueHub.or(targetRedHub).negate().and(this::isRobotTopHalf);
  private final Trigger feedBottomCorner =
          targetBlueHub.or(targetRedHub).or(feedTopCorner).negate();

  private final Translation2d topCorner = new Translation2d(1.5, 6.8);
  private final Translation2d bottomCorner = new Translation2d(1.5, 1.5);

  private double velMultiplier = 1;

  private Pose2d target =
          new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero);

  public Superstructure(
          Indexer indexer,
          ShooterPivot shooterPivot,
          Shooter shooter,
          IntakeRollers intakeRollers,
          LinearSlide linearSlide,
          Feeder feeder,
          ShotCalculator shotCalculator,
          Supplier<Pose2d> robotPoseSupplier) {
    this.indexer = indexer;
    this.shooterPivot = shooterPivot;
    this.shooter = shooter;
    this.intakeRollers = intakeRollers;
    this.linearSlide = linearSlide;
    this.feeder = feeder;
    this.shotCalculator = shotCalculator;
    this.robotPoseSupplier = robotPoseSupplier;

    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }
    for (StructureState state : StructureState.values()) {
      prevStateTriggers.put(state, new Trigger(() -> this.prevState == state));
    }

    configStateTransitions();
  }

  public void configStateTransitions() {

    targetBlueHub.onTrue(changeTarget(FieldConstants.Hub.topCenterPoint.toTranslation2d()));

    targetRedHub.onTrue(changeTarget(FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()));

    targetBlueHub.or(targetRedHub).onTrue(this.setState(StructureState.REV));
    targetBlueHub.or(targetRedHub).onFalse(shooter.off());

    targetBlueHub
            .or(targetRedHub)
            .negate()
            .and(stateTriggers.get(StructureState.REV))
            .onTrue(this.setState(StructureState.IDLE));
    targetBlueHub
            .or(targetRedHub)
            .negate()
            .and(stateTriggers.get(StructureState.REV_AND_INTAKE))
            .onTrue(this.setState(StructureState.INTAKE));

    feedTopCorner.onTrue(
            changeTarget(
                    () -> getAllianceBlue() ? topCorner : ChoreoAllianceFlipUtil.flip(bottomCorner)));
    feedBottomCorner.onTrue(
            changeTarget(
                    () -> getAllianceBlue() ? bottomCorner : ChoreoAllianceFlipUtil.flip(topCorner)));

    targetRedHub
            .or(targetBlueHub)
            .and(DriverStation::isEnabled)
            .and(
                    stateTriggers
                            .get(StructureState.SHOOT)
                            .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
                            .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
            .whileTrue(shooterPivot.shootHub(shotCalculator::getDistance));
    feedTopCorner
            .or(feedBottomCorner)
            .and(DriverStation::isEnabled)
            .and(
                    stateTriggers
                            .get(StructureState.SHOOT)
                            .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
                            .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
            .whileTrue(shooterPivot.feedCorner(shotCalculator::getDistance));

    stateTriggers
            .get(StructureState.SHOOT)
            .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
            .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
            .and(targetRedHub.or(targetBlueHub))
            .onTrue(shooter.shootHub(shotCalculator::getDistance, () -> velMultiplier));
    stateTriggers
            .get(StructureState.SHOOT)
            .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
            .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
            .and(feedTopCorner.or(feedBottomCorner))
            .onTrue(shooter.feedCorner(shotCalculator::getDistance));

    stateTriggers
            .get(StructureState.SHOOT)
            .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
            .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
            .and(shooter.reachedVelocity)
            .debounce(.2)
            .onTrue(indexer.setIndexVel())
            .onTrue(feeder.setFeedVel());

    stateTriggers
            .get(StructureState.SHOOT)
            .and(prevStateTriggers.get(StructureState.INTAKE))
            .or(
                    stateTriggers
                            .get(StructureState.INTAKE)
                            .and(prevStateTriggers.get(StructureState.SHOOT)))
            .or(
                    stateTriggers
                            .get(StructureState.SHOOT)
                            .and(prevStateTriggers.get(StructureState.REV_AND_INTAKE)))
            .or(
                    stateTriggers
                            .get(StructureState.SHOOT)
                            .and(prevStateTriggers.get(StructureState.JITTER_AND_INTAKE)))
            .onTrue(this.setState(StructureState.SHOOT_AND_INTAKE));

    stateTriggers
            .get(StructureState.INTAKE)
            .or(stateTriggers.get(StructureState.SHOOT_AND_INTAKE))
            .or(stateTriggers.get(StructureState.REV_AND_INTAKE))
            .or(stateTriggers.get(StructureState.JITTER_AND_INTAKE))
            .onTrue(intakeRollers.setVoltage(8))
            .onTrue(linearSlide.goToGroundIntake());

    stateTriggers
            .get(StructureState.IDLE)
            .onTrue(intakeRollers.off())
            .onTrue(shooter.off())
            .onTrue(indexer.off())
            .onTrue(feeder.off())
            .onTrue(linearSlide.off());

    // Kills all subsystems
    stateTriggers
            .get(StructureState.CANCEL_ALL)
            .onTrue(intakeRollers.off())
            .onTrue(linearSlide.off())
            .onTrue(shooter.off())
            .onTrue(shooterPivot.off())
            .onTrue(indexer.off())
            .onTrue(feeder.off());

    stateTriggers
            .get(StructureState.HOME)
            .onTrue(linearSlide.goToStow())
            .onTrue(shooterPivot.setPosition(0));

    stateTriggers
            .get(StructureState.JITTER)
            .or(stateTriggers.get(StructureState.JITTER_AND_SHOOT))
            .or(stateTriggers.get(StructureState.JITTER_AND_INTAKE))
            .onTrue(linearSlide.jitterIntake());

    stateTriggers
            .get(StructureState.JITTER)
            .and(
                    prevStateTriggers
                            .get(StructureState.SHOOT)
                            .or(prevStateTriggers.get(StructureState.SHOOT_AND_INTAKE)))
            .onTrue(this.setState(StructureState.JITTER_AND_SHOOT));
    stateTriggers
            .get(StructureState.JITTER)
            .and(
                    prevStateTriggers
                            .get(StructureState.INTAKE))
            .onTrue(this.setState(StructureState.JITTER_AND_INTAKE));

    stateTriggers
            .get(StructureState.REV)
            .and(prevStateTriggers.get(StructureState.INTAKE))
            .or(stateTriggers.get(StructureState.INTAKE).and(prevStateTriggers.get(StructureState.REV)))
            .onTrue(this.setState(StructureState.REV_AND_INTAKE));

    stateTriggers
            .get(StructureState.REV)
            .or(stateTriggers.get(StructureState.REV_AND_INTAKE))
            .onTrue(feeder.off())
            .whileTrue(shooter.shootHub(shotCalculator::getDistance, () -> velMultiplier));

    stateTriggers
            .get(StructureState.INTAKE)
            .or(stateTriggers.get(StructureState.IDLE))
            .or(stateTriggers.get(StructureState.CANCEL_ALL))
            .or(stateTriggers.get(StructureState.HOME))
            .or(stateTriggers.get(StructureState.REV))
            .or(stateTriggers.get(StructureState.REV_AND_INTAKE))
            .or(stateTriggers.get(StructureState.JITTER))
            .or(stateTriggers.get(StructureState.JITTER_AND_INTAKE))
            .onTrue(shooterPivot.setPosition(0));
  }

  // call manually
  public void periodic() {

    Logger.recordOutput("Superstructure/State", this.state.toString());
    Logger.recordOutput("Superstructure/PrevState", this.prevState.toString());
    Logger.recordOutput("Superstructure/StateTime", this.stateTimer.get());

    Logger.recordOutput("Superstructure/VelMultiplier", this.velMultiplier);

    Logger.recordOutput("Superstructure/Target", target);

    LoggedTracer.record(this.getClass().getSimpleName());
  }

  private Command changeTarget(Translation2d target) {
    return changeTarget(() -> target);
  }

  private Command changeTarget(Supplier<Translation2d> target) {
    return Commands.runOnce(
                    () -> {
                      this.target = new Pose2d(target.get(), Rotation2d.kZero);
                      shotCalculator.setTarget(target.get());
                    })
            .ignoringDisable(true);
  }

  private boolean getAllianceBlue() {
    return DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);
  }

  private boolean targetBlueHub() {
    return (robotPoseSupplier.get().getX() < 4 && getAllianceBlue());
  }

  private boolean targetRedHub() {
    return (robotPoseSupplier.get().getX() > 12.5 && !getAllianceBlue());
  }

  private boolean isRobotTopHalf() {
    return robotPoseSupplier.get().getY() > 4;
  }

  public Command setState(StructureState state) {
    return Commands.runOnce(
            () -> {
              this.prevState = this.state == state ? this.prevState : this.state;
              this.state = state;
              this.stateTimer.restart();
            });
  }

  public Command addShootMultiplier(double amt) {
    return Commands.runOnce(() -> this.velMultiplier += amt);
  }

  public StructureState getState() {
    return this.state;
  }

  public StructureState getPrevState() {
    return this.prevState;
  }
}
