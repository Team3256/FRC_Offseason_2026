// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {

  private static double phaseDelay;

  private static final double funnyNum = 0;
  private static final InterpolatingDoubleTreeMap timeOfFlightMapHub =
      new InterpolatingDoubleTreeMap() {
        {
          put(1.322, 0.88);
          put(1.826, 1.0);
          put(2.247, 1.01);
          put(2.765, 1.1);
          put(3.135, 1.0);
          put(3.501, 1.02);
          put(3.901, 1.09);
          put(4.017, 1.12);
          put(4.560, 1.2);
          //          put(4.857, 1.27);
          //          put(5.616, 1.2);
          put(5.837, 1.27);
        }
      };
  private static final InterpolatingDoubleTreeMap timeOfFlightMapFeed =
      new InterpolatingDoubleTreeMap() {
        {
          put(4.38, 1.12);
          put(6.11, 1.2);
          put(8.0, 1.3);
          put(9.42, 1.3);
          put(10.8, 1.55);
          put(12.834, 1.7);

          // fake data
          put(14.0, 1.8);
          put(15.0, 1.9);
          put(16.0, 2.0);
          put(17.0, 2.1);
        }
      };
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;
  private final Transform2d robotToShooter;

  private Pose2d lookaheadPose;

  private Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();

  public ShotCalculator(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier,
      Transform2d robotToShooter) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;
    this.robotToShooter = robotToShooter;
    phaseDelay = 0.05;
  }

  // Suppliers for pose and velocity
  /** Call this in a periodic loop */
  public void periodic() {
    if (robotPoseSupplier == null || robotVelocitySupplier == null) return;

    InterpolatingDoubleTreeMap timeOfFlightMap = timeOfFlightMapHub;
    if (!(target.equals(FieldConstants.Hub.topCenterPoint.toTranslation2d())
        || target.equals(FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()))) {
      timeOfFlightMap = timeOfFlightMapFeed;
    }

    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds robotVelocity = robotVelocitySupplier.get();

    // Phase delay
    Pose2d estimatedPose =
        robotPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

    Pose2d shooterPosition = estimatedPose.transformBy(robotToShooter);

    // Target

    // Distance from shooter to target
    double shooterToTargetDistance = target.getDistance(shooterPosition.getTranslation());

    // Field-relative shooter velocity
    double robotAngle = estimatedPose.getRotation().getRadians();
    double shooterVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToShooter.getY() * Math.cos(robotAngle)
                    - robotToShooter.getX() * Math.sin(robotAngle));
    double shooterVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToShooter.getX() * Math.cos(robotAngle)
                    - robotToShooter.getY() * Math.sin(robotAngle));

    Pose2d lookaheadPose = shooterPosition;
    double currentDistance = shooterToTargetDistance; // starting estimate

    for (int i = 0; i < 20; i++) {
      double timeOfFlight = timeOfFlightMap.get(currentDistance) - funnyNum;

      double offsetX = shooterVelocityX * timeOfFlight;
      double offsetY = shooterVelocityY * timeOfFlight;

      // update lookahead pose
      lookaheadPose =
          new Pose2d(
              shooterPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              shooterPosition.getRotation());

      // let's update the distance this time guys...
      currentDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    this.lookaheadPose = lookaheadPose;

    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", shooterToTargetDistance);

    this.driveAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    
  }

  private Rotation2d driveAngle = Rotation2d.kZero;

  public Rotation2d getDriveAngle() {
    return driveAngle;
  }

  public Pose2d getLookaheadPose() {
    return lookaheadPose;
  }

  public double getDistance() {
    return lookaheadPose.getTranslation().getDistance(target);
  }

  public void setTarget(Translation2d target) {
    this.target = target;
  }
}
