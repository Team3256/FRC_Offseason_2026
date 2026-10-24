package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.Arrays;
import mayhemlib.trajectory.MayhemTrajectory;
import mayhemlib.trajectory.TrajectoryLoader;
import org.junit.jupiter.api.Test;

/**
 * Every path in src/main/deploy/mayhem must be solved and loadable by MayhemLib. Fails the same way
 * the robot would ("Trajectory 'X' has not been generated yet") if a path was edited in the Mayhem
 * app but never regenerated, or if the file format is newer than the vendored MayhemLib.
 */
class MayhemTrajectoriesTest {
  private static final File DEPLOY_DIR = new File("src/main/deploy/mayhem");

  @Test
  void everyTrajectoryLoads() {
    File[] files = DEPLOY_DIR.listFiles((dir, name) -> name.endsWith(TrajectoryLoader.EXTENSION));
    assertNotNull(files, DEPLOY_DIR + " is missing");
    Arrays.sort(files);

    for (File file : files) {
      String name = file.getName().substring(0, file.getName().length() - 6);
      MayhemTrajectory traj = TrajectoryLoader.load(file);

      assertEquals(name, traj.name(), "file name should match the trajectory name");
      assertFalse(traj.samples().isEmpty(), name + " has no samples");
      assertTrue(traj.totalTime() > 0, name + " has no duration");
      assertNotNull(traj.recovery(), name + " is missing recovery data");
      assertTrue(
          Double.isFinite(traj.initialPose().getX()) && Double.isFinite(traj.finalPose().getX()),
          name + " has a non-finite pose");
    }
  }
}
