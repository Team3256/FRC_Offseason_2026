// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>

#include "mayhem/Auto.h"

class Superstructure;
class ShotCalculator;
namespace subsystems {
class CommandSwerveDrivetrain;
}

/** Shared pieces of the Mayhem autos. */
class MayhemAutosBase {
 protected:
  MayhemAutosBase(mayhem::AutoFactory& factory, Superstructure& superstructure,
                  subsystems::CommandSwerveDrivetrain* drivetrain, ShotCalculator* shotCalculator)
      : m_factory{factory},
        m_superstructure{superstructure},
        m_drivetrain{drivetrain},
        m_shotCalculator{shotCalculator} {}

  /** Face the hub (3 degree tolerance), then SHOOT; gives up after 3.5 s. */
  frc2::CommandPtr AlignAndShoot();

  mayhem::AutoFactory& m_factory;
  Superstructure& m_superstructure;
  subsystems::CommandSwerveDrivetrain* m_drivetrain;
  ShotCalculator* m_shotCalculator;
};

/** Trench sweep autos (Outpost = bottom trench, Depot = top trench). */
class SweepMayhemAutos : public MayhemAutosBase {
 public:
  SweepMayhemAutos(mayhem::AutoFactory& factory, Superstructure& superstructure,
                   subsystems::CommandSwerveDrivetrain& drivetrain, ShotCalculator& shotCalculator)
      : MayhemAutosBase(factory, superstructure, &drivetrain, &shotCalculator) {}

  /** 1. Bottom Trench: Part 1 only */
  std::shared_ptr<mayhem::AutoRoutine> BottomTrenchPart1Only();
  /** 2. Bottom Trench: Parts 1 and 2 */
  std::shared_ptr<mayhem::AutoRoutine> BottomTrenchPart1And2();
  /** 3. Top Trench: Part 1 only */
  std::shared_ptr<mayhem::AutoRoutine> TopTrenchPart1Only();
  /** 4. Top Trench: Parts 1 and 2 */
  std::shared_ptr<mayhem::AutoRoutine> TopTrenchPart1And2();
};

/** Mayhem version of the top trench sweep depot auto (paths in deploy/mayhem). */
class SweepDepotMayhemAutos : public MayhemAutosBase {
 public:
  SweepDepotMayhemAutos(mayhem::AutoFactory& factory, Superstructure& superstructure,
                        subsystems::CommandSwerveDrivetrain& drivetrain,
                        ShotCalculator& shotCalculator)
      : MayhemAutosBase(factory, superstructure, &drivetrain, &shotCalculator) {}

  std::shared_ptr<mayhem::AutoRoutine> TopTrenchSweepDepot();
};

/** Depot-only autos (not on the chooser, kept from the Java code). */
class DepotOnlyMayhemAutos : public MayhemAutosBase {
 public:
  DepotOnlyMayhemAutos(mayhem::AutoFactory& factory, Superstructure& superstructure)
      : MayhemAutosBase(factory, superstructure, nullptr, nullptr) {}

  std::shared_ptr<mayhem::AutoRoutine> TrenchDepotOnlyMayhemAuto();
  std::shared_ptr<mayhem::AutoRoutine> BumpDepotOnlyMayhemAuto();
  std::shared_ptr<mayhem::AutoRoutine> HubDepotOnlyMayhemAuto();
};
