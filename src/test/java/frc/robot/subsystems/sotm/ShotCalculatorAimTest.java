// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sotm;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class ShotCalculatorAimTest {
  private static final double kTol = Math.toRadians(5);

  @Test
  void aimedWithinTolerance() {
    assertTrue(ShotCalculator.isAimed(Rotation2d.fromDegrees(92), Rotation2d.fromDegrees(90), kTol));
  }

  @Test
  void notAimedOutsideTolerance() {
    assertFalse(
        ShotCalculator.isAimed(Rotation2d.fromDegrees(100), Rotation2d.fromDegrees(90), kTol));
  }

  @Test
  void wrapsAroundPlusMinus180() {
    assertTrue(
        ShotCalculator.isAimed(Rotation2d.fromDegrees(179), Rotation2d.fromDegrees(-179), kTol));
    assertFalse(
        ShotCalculator.isAimed(Rotation2d.fromDegrees(170), Rotation2d.fromDegrees(-170), kTol));
  }
}
