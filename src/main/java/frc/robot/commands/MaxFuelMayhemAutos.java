// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import mayhemlib.auto.AutoFactory;
import mayhemlib.auto.AutoRoutine;
import mayhemlib.auto.AutoTrajectory;

/**
 * Max-fuel autos modelled on what the best dumper robots ran in 2026 (see docs/mayhem-autos). The
 * paths live in {@code src/main/deploy/mayhem}. Every path is authored for blue and flipped for red
 * by MayhemLib.
 *
 * <p>The robot parks while it dumps, so the waits below are the one thing to tune on the real
 * robot: they are how long the hopper takes to empty (about 1 s per 10 fuel at the shooter's rate)
 * and how long the intake needs to fill from the depot.
 */
public class MaxFuelMayhemAutos {
  /** Seconds the intake sits in the depot before leaving (24 fuel). */
  private static final double kDepotIntakeSeconds = 2.5;

  /** Seconds parked after the first neutral zone load, while the hopper empties. */
  private static final double kFirstDumpSeconds = 4.5;

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public MaxFuelMayhemAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  /** Top trench out, two loads from the neutral zone, back over the top bump to dump each time. */
  public AutoRoutine nzDoubleTop() {
    return nzDouble("nzDoubleTop", "nzDoubleTopPt1", "nzDoubleTopPt2");
  }

  /** Same as {@link #nzDoubleTop()} on the bottom side of the field. */
  public AutoRoutine nzDoubleBottom() {
    return nzDouble("nzDoubleBottom", "nzDoubleBottomPt1", "nzDoubleBottomPt2");
  }

  /** Preload and depot (32 fuel), then a full load from the neutral zone through the top trench. */
  public AutoRoutine depotTrenchSweep() {
    return depotThen("depotTrenchSweep", "depotTrenchSweepPt2");
  }

  /** Preload and depot (32 fuel), then a full load from the neutral zone over the top bump. */
  public AutoRoutine depotBumpSweep() {
    return depotThen("depotBumpSweep", "depotBumpSweepPt2");
  }

  /** Preload and depot only (32 fuel). Never enters the neutral zone. */
  public AutoRoutine depotOnly() {
    final AutoRoutine routine = m_factory.newRoutine("depotOnly");
    final AutoTrajectory depot = routine.trajectory("depotPt1");
    final AutoTrajectory dump = routine.trajectory("depotDumpPt2");
    routine.active().onTrue(depot.resetOdometry().andThen(depot.cmd()));
    depot.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    depot.done().onTrue(Commands.waitSeconds(kDepotIntakeSeconds).andThen(dump.cmd()));
    dump.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  private AutoRoutine nzDouble(String name, String pt1Name, String pt2Name) {
    final AutoRoutine routine = m_factory.newRoutine(name);
    final AutoTrajectory pt1 = routine.trajectory(pt1Name);
    final AutoTrajectory pt2 = routine.trajectory(pt2Name);
    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));
    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt1.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));
    pt1.done().onTrue(Commands.waitSeconds(kFirstDumpSeconds).andThen(pt2.cmd()));

    pt2.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt2.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt2.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  private AutoRoutine depotThen(String name, String sweepName) {
    final AutoRoutine routine = m_factory.newRoutine(name);
    final AutoTrajectory depot = routine.trajectory("depotPt1");
    final AutoTrajectory sweep = routine.trajectory(sweepName);
    routine.active().onTrue(depot.resetOdometry().andThen(depot.cmd()));
    depot.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    depot.done().onTrue(Commands.waitSeconds(kDepotIntakeSeconds).andThen(sweep.cmd()));

    // shoot the preload and depot fuel while driving out of the depot, still inside the alliance
    // zone
    sweep.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));
    sweep.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    sweep.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    sweep.atTime("idle2").onTrue(m_superstructure.setState(StructureState.IDLE));
    sweep.atTime("shoot2").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }
}
