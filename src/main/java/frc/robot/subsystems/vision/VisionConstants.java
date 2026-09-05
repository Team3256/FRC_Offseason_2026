// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String frontRightCam = "frontRight";
  public static String frontLeftCam = "frontLeft";
  public static String backCam = "back";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToFrontRightCam =
      new Transform3d(
          Units.inchesToMeters(-2.500),
          Units.inchesToMeters(-13.667),
          Units.inchesToMeters(7.677),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)));

  public static Transform3d robotToFrontLeftCam =
      new Transform3d(
          Units.inchesToMeters(-2.500),
          Units.inchesToMeters(13.677),
          Units.inchesToMeters(7.677),
          new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(90)));

  public static Transform3d robotToBackCam =
      new Transform3d(
          Units.inchesToMeters(-13.783),
          Units.inchesToMeters(0),
          Units.inchesToMeters(-9.802),
          new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(180)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.99; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1, // Camera 0
        1, // Camera 1
        1,
        1 // Camera 3
      };

  public static List<Short> nonHub =
      List.of(
          (short) 1,
          (short) 6,
          (short) 7,
          (short) 12,
          (short) 13,
          (short) 14,
          (short) 15,
          (short) 16,
          (short) 17,
          (short) 22,
          (short) 23,
          (short) 28,
          (short) 29,
          (short) 30,
          (short) 31,
          (short) 32);
}
