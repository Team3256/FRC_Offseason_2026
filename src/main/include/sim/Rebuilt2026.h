// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <vector>

#include "mayhem/sim/field.hpp"

/**
 * The 2026 REBUILT field for mayhem::sim, from FieldConstants (the welded AprilTag layout and the
 * game manual dimensions). Blue-origin field frame; the red half is the blue half rotated 180
 * degrees about the field centre.
 */
namespace rebuilt {

/** Trench overhead arm: robots taller than this hit it (game manual: 22.25 in opening). */
inline constexpr double kTrenchClearanceM = 22.25 * 0.0254;
/** Top of the hub funnel, where FUEL falls in. */
inline constexpr double kHubTopM = 72.0 * 0.0254;
/** Hub funnel opening radius (41.7 in inner width) and the height where it stops being a goal. */
inline constexpr double kHubOpeningRadiusM = 41.7 / 2 * 0.0254;
inline constexpr double kHubGoalBottomM = 1.40;

/** Obstacle names (prefixes) a test can match ContactEvents against. */
inline constexpr const char* kPerimeterName = "perimeter";
inline constexpr const char* kTrenchArmPrefix = "trench_arm";
inline constexpr const char* kTrenchPostPrefix = "trench_post";
inline constexpr const char* kHubPrefix = "hub";
inline constexpr const char* kTowerPrefix = "tower";

/** Builds the carpet features (bumps, depots) and the obstacles. */
void BuildField(mayhem::sim::Terrain& terrain, mayhem::sim::Obstacles& obstacles);

/**
 * FUEL staged at the start of a match (from AdvantageScope's 2026 field model): the 12 x 30 grid
 * in the neutral zone and 24 in each depot.
 */
std::vector<std::array<double, 2>> StagedFuel();

/** Hub centres (blue, red). */
std::array<double, 2> HubCenter(bool red);

}  // namespace rebuilt
