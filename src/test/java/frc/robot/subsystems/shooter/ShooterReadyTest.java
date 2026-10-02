// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class ShooterReadyTest {
  /** Reports whatever velocity the test sets, on all four motors. */
  private static class StubShooterIO implements ShooterIO {
    double velocity = 0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      java.util.Arrays.fill(inputs.shooterMotorVelocitys, velocity);
    }
  }

  @AfterEach
  void cleanUp() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void stoppedShooterThatWasNeverToldToSpinIsNotReady() {
    StubShooterIO io = new StubShooterIO();
    Shooter shooter = new Shooter(true, io);
    shooter.periodic();
    assertFalse(shooter.reachedVelocity());
  }

  @Test
  void readyOnlyOnceAtRequestedSpeed() {
    StubShooterIO io = new StubShooterIO();
    Shooter shooter = new Shooter(true, io);

    CommandScheduler.getInstance().schedule(shooter.setVelocity(40).ignoringDisable(true));
    CommandScheduler.getInstance().run();
    shooter.periodic();
    assertFalse(shooter.reachedVelocity(), "still stopped");

    io.velocity = 38;
    shooter.periodic();
    assertTrue(shooter.reachedVelocity(), "within 5 of 40");

    io.velocity = 20;
    shooter.periodic();
    assertFalse(shooter.reachedVelocity(), "dipped well below request");
  }
}
