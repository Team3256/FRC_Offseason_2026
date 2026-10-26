// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.TrenchAssistConstants;
import frc.robot.subsystems.swerve.TrenchAssist;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * Runs the real RobotContainer and the CTRE swerve simulation.
 *
 * <p>The always-on tests check the assist's decisions through the real drivetrain (auto, the
 * dashboard switch, red and blue alliance). The tests that drive the simulated robot physically
 * through the trenches are opt-in because they take a while ({@code TRENCH_SIM=true ./gradlew
 * test}).
 */
class TrenchAssistSimTest {
  private static final int kDriverPort = Constants.ControllerConstants.kDriverControllerPort;
  private static final double kRobotHalfWidth = 0.42;
  private static final double kLoopSeconds = 0.02;
  // Operator left stick forward; squared by the drive code, so this is ~2.9 m/s
  private static final double kStick = -0.75;
  private static final double kClearance =
      FieldConstants.LeftTrench.openingWidth / 2.0 - kRobotHalfWidth + 0.01;

  private static RobotContainer container;
  private static CommandSwerveDrivetrain drivetrain;

  @BeforeAll
  static void setUp() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    container = new RobotContainer();
    Field field = RobotContainer.class.getDeclaredField("drivetrain");
    field.setAccessible(true);
    drivetrain = (CommandSwerveDrivetrain) field.get(container);
  }

  @AfterAll
  static void tearDown() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    HAL.shutdown();
  }

  private static void loop() throws InterruptedException {
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().run();
    container.periodic();
    Thread.sleep((long) (kLoopSeconds * 1000));
  }

  private static void setAlliance(AllianceStationID station) throws InterruptedException {
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
    for (int i = 0; i < 10; i++) {
      loop();
    }
  }

  private static void stick(double leftX, double leftY, double rightX) {
    DriverStationSim.setJoystickAxisCount(kDriverPort, 6);
    DriverStationSim.setJoystickAxis(kDriverPort, 0, leftX);
    DriverStationSim.setJoystickAxis(kDriverPort, 1, leftY);
    DriverStationSim.setJoystickAxis(kDriverPort, 4, rightX);
    DriverStationSim.notifyNewData();
  }

  private static void enable(boolean autonomous) {
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  private static void disable() {
    stick(0, 0, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  /** Offsets from the lane center a robot can actually be at: the field wall is on one side. */
  private static double[] reachableOffsets(double laneY) {
    double towardFieldCenter = laneY < FieldConstants.fieldWidth / 2.0 ? 1.0 : -1.0;
    return new double[] {0.35 * towardFieldCenter, -0.2 * towardFieldCenter};
  }

  private static boolean inTrench(Pose2d pose, double trenchX) {
    return Math.abs(pose.getX() - trenchX)
        <= FieldConstants.LeftTrench.depth / 2.0 + TrenchAssistConstants.kRobotHalfLength;
  }

  /**
   * Drives with the stick straight forward. The robot first gets up to speed in a clear lane
   * (a start from rest is not what a driver does), then at {@code perturbX} it is nudged to {@code
   * perturbed} while still moving, as if the driver had lined up badly. Returns the worst lateral
   * offset (90th percentile of the samples) and the most its heading moved from the nudged one,
   * while the robot is between the walls of {@code trenchX}.
   *
   * <p>Only lateral centering is checked here; heading is covered by TrenchAssistTest.
   */
  private static double[] driveForward(
      Pose2d rollStart, double perturbX, Pose2d perturbed, double trenchX, double laneY)
      throws InterruptedException {
    stick(0, 0, 0);
    enable(false);
    drivetrain.resetPose(rollStart);
    for (int i = 0; i < 10; i++) {
      loop();
    }

    boolean forwardIsPlusX = perturbX > rollStart.getX();
    stick(0, kStick, 0); // forward on the operator's stick
    boolean perturbedYet = false;
    boolean reachedTrench = false;
    List<Double> offsets = new ArrayList<>();
    double headingDrift = 0;
    for (int i = 0; i < 8.0 / kLoopSeconds; i++) {
      loop();
      Pose2d pose = drivetrain.getState().Pose;
            if (!perturbedYet) {
        if (forwardIsPlusX ? pose.getX() >= perturbX : pose.getX() <= perturbX) {
          drivetrain.resetPose(perturbed);
          perturbedYet = true;
        }
        continue;
      }
      if (inTrench(pose, trenchX)) {
        reachedTrench = true;
        offsets.add(Math.abs(pose.getY() - laneY));
        headingDrift =
            Math.max(
                headingDrift,
                Math.abs(
                    MathUtil.angleModulus(
                        pose.getRotation().getRadians() - perturbed.getRotation().getRadians())));
      } else if (reachedTrench) {
        break; // through and out the other side
      }
    }
    disable();
    assertTrue(perturbedYet, "robot never reached the nudge point, the sim is not driving");
    assertTrue(reachedTrench, "robot never reached the trench");
    Collections.sort(offsets);
    double offset = offsets.get((int) Math.min(offsets.size() - 1, offsets.size() * 0.9));
    System.out.printf(
        "TRENCH-SIM nudged to %s, trench x=%.2f: offset %.3f m, heading changed up to %.0f deg (%d samples)%n",
        perturbed, trenchX, offset, Math.toDegrees(headingDrift), offsets.size());
    return new double[] {offset, headingDrift};
  }

  @Test
  @EnabledIfEnvironmentVariable(named = "TRENCH_SIM", matches = "true")
  void driverPushingForwardGoesThroughTrenchesCentered() throws Exception {
    double[] laneYs = TrenchAssist.laneCenterYs();
    double[] trenchXs = TrenchAssist.trenchCenterXs();

    // Blue alliance: forward is +x. Roll through the near trench, get nudged in the neutral zone,
    // then go through the far trench
    setAlliance(AllianceStationID.Blue1);
    for (double laneY : laneYs) {
      for (double offset : reachableOffsets(laneY)) {
        Pose2d perturbed = new Pose2d(6.5, laneY + offset, Rotation2d.fromDegrees(offset * 80));
        double[] worst =
            driveForward(
                new Pose2d(0.8, laneY, Rotation2d.kZero), 6.5, perturbed, trenchXs[1], laneY);
        assertTrue(worst[0] < kClearance, "blue: offset " + worst[0]);
      }
    }

    // Red alliance: forward on the stick is -x on the field
    setAlliance(AllianceStationID.Red1);
    for (double laneY : laneYs) {
      for (double offset : reachableOffsets(laneY)) {
        Pose2d perturbed =
            new Pose2d(10.0, laneY + offset, Rotation2d.fromDegrees(180 + offset * 80));
        double[] worst =
            driveForward(
                new Pose2d(15.7, laneY, Rotation2d.k180deg), 10.0, perturbed, trenchXs[0], laneY);
        assertTrue(worst[0] < kClearance, "red: offset " + worst[0]);
      }
    }
  }

  @Test
  @EnabledIfEnvironmentVariable(named = "TRENCH_SIM", matches = "true")
  void robotKeepsItsOffsetWhenDisabledFromTheDashboard() throws Exception {
    setAlliance(AllianceStationID.Blue1);
    double laneY = TrenchAssist.laneCenterYs()[0];
    double trenchX = TrenchAssist.trenchCenterXs()[1];
    Pose2d perturbed = new Pose2d(6.5, laneY + 0.35, Rotation2d.kZero);
    Pose2d rollStart = new Pose2d(0.8, laneY, Rotation2d.kZero);

    // Same stick, but the assist must not run: the robot just keeps its offset
    SmartDashboard.putBoolean("TrenchAssist/Enabled", false);
    double off = driveForward(rollStart, 6.5, perturbed, trenchX, laneY)[0];
    SmartDashboard.putBoolean("TrenchAssist/Enabled", true);
    assertTrue(off > 0.3, "assist ran while disabled on the dashboard: " + off);

    // Control: same run with the assist on does center
    double on = driveForward(rollStart, 6.5, perturbed, trenchX, laneY)[0];
    assertTrue(on < kClearance, "control run did not center: " + on);
  }

  @Test
  void doesNothingInAuto() throws Exception {
    setAlliance(AllianceStationID.Blue1);
    double laneY = TrenchAssist.laneCenterYs()[0];
    double trenchX = TrenchAssist.trenchCenterXs()[1];
    var driver = new ChassisSpeeds(3.0, 0.2, 0);

    // Lined up to go through a trench, off-center, driver pushing forward
    stick(0, 0, 0);
    enable(false);
    drivetrain.resetPose(new Pose2d(trenchX - 1.8, laneY + 0.35, Rotation2d.kZero));
    for (int i = 0; i < 10; i++) {
      loop();
    }
    boolean acted = false;
    for (int i = 0; i < 10; i++) {
      loop();
      acted |= drivetrain.applyTrenchAssist(driver).vyMetersPerSecond != driver.vyMetersPerSecond;
    }
    assertTrue(acted, "control: assist did not act in teleop");

    enable(true);
    for (int i = 0; i < 5; i++) {
      loop();
    }
    for (int i = 0; i < 10; i++) {
      loop();
      var auto = drivetrain.applyTrenchAssist(driver);
      assertTrue(
          auto.vyMetersPerSecond == driver.vyMetersPerSecond
              && auto.omegaRadiansPerSecond == driver.omegaRadiansPerSecond,
          "assist modified the speeds in auto: " + auto);
    }
    disable();
  }

  @Test
  @EnabledIfEnvironmentVariable(named = "TRENCH_SIM", matches = "true")
  void aSidewaysRobotGoesThroughCenteredWithoutBeingTurned() throws Exception {
    double[] laneYs = TrenchAssist.laneCenterYs();
    double[] trenchXs = TrenchAssist.trenchCenterXs();
    setAlliance(AllianceStationID.Blue1);
    for (double laneY : laneYs) {
      for (double heading : new double[] {90, -90}) {
        double offset = reachableOffsets(laneY)[0];
        Pose2d perturbed = new Pose2d(6.5, laneY + offset, Rotation2d.fromDegrees(heading));
        double[] worst =
            driveForward(
                new Pose2d(0.8, laneY, Rotation2d.kZero), 6.5, perturbed, trenchXs[1], laneY);
        assertTrue(worst[0] < kClearance, "sideways: offset " + worst[0]);
        assertTrue(
            worst[1] < Math.toRadians(10),
            "sideways robot was turned by " + Math.toDegrees(worst[1]) + " deg");
      }
    }
  }

  /** Puts the (idle) robot at a pose and waits for the pose estimate to settle there. */
  private static void placeRobot(Pose2d pose) throws InterruptedException {
    stick(0, 0, 0);
    enable(false);
    drivetrain.resetPose(pose);
    for (int i = 0; i < 15; i++) {
      loop();
    }
  }

  @Test
  void pushesTowardTheLaneOnBothAlliances() throws Exception {
    double laneY = TrenchAssist.laneCenterYs()[0];
    double trenchX = TrenchAssist.trenchCenterXs()[0];
    // Operator stick straight forward (no sideways input) at full speed
    var forward = new ChassisSpeeds(4.0, 0.0, 0.0);

    // Blue: the operator's frame is the field frame; robot above the lane, so it must go -y
    setAlliance(AllianceStationID.Blue1);
    placeRobot(new Pose2d(trenchX - 2.0, laneY + 0.3, Rotation2d.kZero));
    var blue = drivetrain.applyTrenchAssist(forward);
    assertTrue(blue.vyMetersPerSecond < -0.3, "blue should be pushed toward -y: " + blue);

    // Red: the operator's frame is rotated 180 degrees, so "forward" is -x on the field and the
    // same push toward -y on the field shows up as +y in the operator's frame
    setAlliance(AllianceStationID.Red1);
    placeRobot(new Pose2d(trenchX + 2.0, laneY + 0.3, Rotation2d.k180deg));
    var red = drivetrain.applyTrenchAssist(forward);
    assertTrue(red.vyMetersPerSecond > 0.3, "red should be pushed toward -y (field): " + red);
  }

  @Test
  void switchedOffByTheDashboard() throws Exception {
    double laneY = TrenchAssist.laneCenterYs()[0];
    double trenchX = TrenchAssist.trenchCenterXs()[0];
    var forward = new ChassisSpeeds(4.0, 0.0, 0.0);

    setAlliance(AllianceStationID.Blue1);
    placeRobot(new Pose2d(trenchX - 2.0, laneY + 0.3, Rotation2d.kZero));
    assertTrue(drivetrain.applyTrenchAssist(forward).vyMetersPerSecond < -0.3, "control");

    SmartDashboard.putBoolean("TrenchAssist/Enabled", false);
    try {
      var off = drivetrain.applyTrenchAssist(forward);
      assertTrue(
          off.vyMetersPerSecond == 0.0 && off.omegaRadiansPerSecond == 0.0,
          "assist ran while switched off: " + off);
    } finally {
      SmartDashboard.putBoolean("TrenchAssist/Enabled", true);
    }
    disable();
  }
}
