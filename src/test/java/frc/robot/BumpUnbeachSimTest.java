package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.PhoenixUtil;
import java.io.File;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import mayhemlib.auto.AutoFactory;
import mayhemlib.sim.FuelPileInjector;
import mayhemlib.trajectory.MayhemTrajectory;
import mayhemlib.trajectory.TrajectoryLoader;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Runs Mayhem paths through the real RobotContainer (CTRE swerve in simulation, the factory's
 * unbeach wired to the simulated Pigeon 2) and checks that auto unbeach only fires for a pile of
 * fuel, never for a bump. The bumps are marked rough terrain in project.mayhem, so every path that
 * crosses one has a rough-terrain span and unbeach is not checked there.
 *
 * <p>The sim has no ramps, so the bump's tilt is written to the Pigeon from the robot's pose, two
 * ways: a 15 degree ramp profile under the wheels, and a pessimistic 12 degrees held for as long as
 * any wheel is on the bump.
 */
class BumpUnbeachSimTest {
  private static final File DEPLOY_DIR = new File("src/main/deploy/mayhem");
  private static final File FIXTURE_DIR = new File("src/test/resources/mayhem");

  // Bump footprints {x0, x1, y0, y1} from project.mayhem (bump-blue-l/r, bump-red-l/r).
  private static final double[][] BUMPS = {
    {4.0618, 5.1878, 4.6286, 6.4849},
    {4.0618, 5.1878, 1.5843, 3.4406},
    {11.3532, 12.4792, 1.5843, 3.4406},
    {11.3532, 12.4792, 4.6286, 6.4849}
  };
  private static final double RAMP = 0.45;
  private static final double HEIGHT = RAMP * Math.tan(Math.toRadians(15));
  private static final double HALF_WHEELBASE = 0.28;
  private static final double[][] WHEELS = {
    {HALF_WHEELBASE, HALF_WHEELBASE},
    {HALF_WHEELBASE, -HALF_WHEELBASE},
    {-HALF_WHEELBASE, HALF_WHEELBASE},
    {-HALF_WHEELBASE, -HALF_WHEELBASE}
  };

  private static CommandSwerveDrivetrain drivetrain;
  private static RobotContainer rc;
  private static AutoFactory factory;

  private record Sample(double x, double y, String state) {}

  @BeforeAll
  static void setup() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    rc = new RobotContainer();
    drivetrain = privateField("drivetrain", CommandSwerveDrivetrain.class);
    factory = privateField("m_mayhemAutoFactory", AutoFactory.class);
  }

  @AfterAll
  static void teardown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    drivetrain.close();
  }

  private static <T> T privateField(String name, Class<T> type) throws Exception {
    Field field = RobotContainer.class.getDeclaredField(name);
    field.setAccessible(true);
    return type.cast(field.get(rc));
  }

  private static double[] bumpAt(double x, double y) {
    for (double[] b : BUMPS) {
      if (x >= b[0] && x <= b[1] && y >= b[2] && y <= b[3]) {
        return b;
      }
    }
    return null;
  }

  private static double height(double x, double y) {
    double[] b = bumpAt(x, y);
    return b == null ? 0 : Math.min(Math.min(x - b[0], b[1] - x) / RAMP, 1.0) * HEIGHT;
  }

  /** Tilt the Pigeon would read with the four wheels on a 15 degree ramp profile. */
  private static void applyRampTilt(Pose2d p) {
    double c = p.getRotation().getCos();
    double s = p.getRotation().getSin();
    double h = HALF_WHEELBASE;
    double front = height(p.getX() + h * c, p.getY() + h * s);
    double rear = height(p.getX() - h * c, p.getY() - h * s);
    double left = height(p.getX() - h * s, p.getY() + h * c);
    double right = height(p.getX() + h * s, p.getY() - h * c);
    var imu = drivetrain.getPigeon2().getSimState();
    // Pigeon 2: positive pitch is nose down, positive roll is left side up
    imu.setPitch(Degrees.of(-Math.toDegrees(Math.atan2(front - rear, 2 * h))));
    imu.setRoll(Degrees.of(Math.toDegrees(Math.atan2(left - right, 2 * h))));
  }

  /** Tilt held at {@code degrees}, leaning downhill, while any wheel is on the bump. */
  private static void applyWorstCaseTilt(Pose2d p, double degrees) {
    double c = p.getRotation().getCos();
    double s = p.getRotation().getSin();
    double lean = 0; // downhill is -x on the way up the bump and +x on the way down
    for (double[] w : WHEELS) {
      double[] b = bumpAt(p.getX() + w[0] * c - w[1] * s, p.getY() + w[0] * s + w[1] * c);
      if (b != null) {
        lean = p.getX() < (b[0] + b[1]) / 2 ? -1 : 1;
      }
    }
    var imu = drivetrain.getPigeon2().getSimState();
    imu.setPitch(Degrees.of(lean * c * degrees));
    imu.setRoll(Degrees.of(lean * s * degrees));
  }

  /** Runs the path in real time (the CTRE sim thread runs on wall-clock time). */
  private static List<Sample> run(MayhemTrajectory traj, boolean worstCase, FuelPileInjector pile)
      throws InterruptedException {
    NetworkTable nt = NetworkTableInstance.getDefault().getTable("Mayhem");
    drivetrain.resetPose(traj.initialPose());
    Thread.sleep(200);
    rc.periodic();
    Command auto = factory.trajectoryCmd(traj);
    CommandScheduler.getInstance().schedule(auto);
    if (pile != null) {
      pile.reset();
      CommandScheduler.getInstance().schedule(pile.command());
    }
    List<Sample> trace = new ArrayList<>();
    long start = System.nanoTime();
    while (CommandScheduler.getInstance().isScheduled(auto)
        && (System.nanoTime() - start) * 1e-9 < traj.totalTime() + 4) {
      DriverStationSim.notifyNewData();
      Pose2d pose = drivetrain.getState().Pose;
      if (pile == null) {
        if (worstCase) {
          applyWorstCaseTilt(pose, 12);
        } else {
          applyRampTilt(pose);
        }
      }
      PhoenixUtil.refreshAll();
      CommandScheduler.getInstance().run();
      rc.periodic();
      trace.add(new Sample(pose.getX(), pose.getY(), nt.getEntry("state").getString("")));
      Thread.sleep(20);
    }
    CommandScheduler.getInstance().cancelAll();
    return trace;
  }

  private static double endError(MayhemTrajectory traj) {
    return drivetrain
        .getState()
        .Pose
        .getTranslation()
        .getDistance(traj.finalPose().getTranslation());
  }

  /** Every path in deploy/mayhem that crosses a bump, plus a fixture so this is never empty. */
  private static List<File> bumpPaths() {
    List<File> files = new ArrayList<>();
    for (File dir : new File[] {FIXTURE_DIR, DEPLOY_DIR}) {
      File[] found = dir.listFiles((d, name) -> name.endsWith(TrajectoryLoader.EXTENSION));
      if (found != null) {
        Arrays.sort(found);
        for (File file : found) {
          if (!TrajectoryLoader.load(file).terrain().isEmpty()) {
            files.add(file);
          }
        }
      }
    }
    return files;
  }

  @Test
  void crossingTheBumpDoesNotUnbeach() throws Exception {
    List<File> files = bumpPaths();
    assertFalse(files.isEmpty(), "no path with a rough-terrain span to drive");

    for (File file : files) {
      MayhemTrajectory traj = TrajectoryLoader.load(file);
      for (boolean worstCase : new boolean[] {false, true}) {
        List<Sample> trace = run(traj, worstCase, null);
        String label = traj.name() + (worstCase ? " (worst-case tilt)" : " (ramp tilt)");
        System.out.printf("%s: end error %.3f m%n", label, endError(traj));
        assertFalse(
            trace.stream().anyMatch(s -> s.state().equals("UNBEACHING")),
            label + " backed out on the bump");
        assertTrue(endError(traj) < 0.15, label + " did not finish the path");
      }
    }
  }

  @Test
  void unbeachStillGetsOffAPileOfFuel() throws Exception {
    MayhemTrajectory traj = TrajectoryLoader.load(new File(FIXTURE_DIR, "topTrenchSweepBump.mtraj"));
    // a pile on the path, well before the bump
    Translation2d center = traj.sampleAt(2.0).getPose().getTranslation();
    FuelPileInjector pile =
        new FuelPileInjector(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            (pitch, roll) -> {
              var imu = drivetrain.getPigeon2().getSimState();
              imu.setPitch(Degrees.of(pitch));
              imu.setRoll(Degrees.of(roll));
            },
            center,
            0.3,
            12);

    List<Sample> trace = run(traj, false, pile);
    System.out.printf("pile at %s: end error %.3f m%n", center, endError(traj));
    assertEquals(1, pile.beachCount(), "the robot reached the pile");
    assertTrue(
        trace.stream().anyMatch(s -> s.state().equals("UNBEACHING")), "entered UNBEACHING");
    assertFalse(pile.isBeached(), "got off the pile");
    assertTrue(endError(traj) < 0.15, "resumed and finished the path");
  }
}
