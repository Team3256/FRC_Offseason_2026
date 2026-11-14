// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "sim/Rebuilt2026.h"

#include <string>

#include "FieldConstants.h"

namespace rebuilt {
namespace {
namespace fc = field_constants;

constexpr double kLength = fc::kFieldLength;
constexpr double kWidth = fc::kFieldWidth;

/** A point on the blue half, rotated onto the red half when {@code red}. */
std::array<double, 2> Side(double x, double y, bool red) {
  return red ? std::array<double, 2>{kLength - x, kWidth - y} : std::array<double, 2>{x, y};
}

/**
 * REBUILT bump cross-section along x (the drive direction): 44.4 in deep, 6.513 in tall, two
 * 15-degree ramps meeting at a ridge (the manual's ramp angle over that depth leaves no flat top).
 */
std::vector<mayhem::sim::Vec2> BumpProfile() {
  const double half = fc::left_bump::depth / 2;
  const double h = fc::left_bump::height;
  return {{-half, 0}, {-half + 0.008, 0.0127}, {0, h}, {half - 0.008, 0.0127}, {half, 0}};
}
}  // namespace

std::array<double, 2> HubCenter(bool red) { return Side(fc::hub::kCenterX, kWidth / 2, red); }

void BuildField(mayhem::sim::Terrain& terrain, mayhem::sim::Obstacles& obstacles) {
  obstacles.SetPerimeter(0, 0, kLength, kWidth, /*chamfer*/ 0, /*wallHeight*/ 0.5, kPerimeterName);
  for (bool red : {false, true}) {
    const std::string side = red ? "_red" : "_blue";
    const double yaw = red ? 3.141592653589793 : 0.0;
    const double hubX = fc::hub::kCenterX;
    const double hubHalf = fc::hub::width / 2;

    // Hub: a solid body up to the bottom of the goal, then four funnel walls around the opening.
    {
      const auto c = Side(hubX, kWidth / 2, red);
      obstacles.AddBox(kHubPrefix + side, c[0], c[1], yaw, fc::hub::width, fc::hub::width, 0,
                       kHubGoalBottomM);
      const double wall = hubHalf - kHubOpeningRadiusM;
      for (int s : {-1, 1}) {
        const auto wx = Side(hubX + s * (hubHalf - wall / 2), kWidth / 2, red);
        obstacles.AddBox(kHubPrefix + side + "_funnel", wx[0], wx[1], yaw, wall, fc::hub::width,
                         kHubGoalBottomM, kHubTopM);
        const auto wy = Side(hubX, kWidth / 2 + s * (hubHalf - wall / 2), red);
        obstacles.AddBox(kHubPrefix + side + "_funnel", wy[0], wy[1], yaw, fc::hub::width, wall,
                         kHubGoalBottomM, kHubTopM);
      }
    }

    // Bumps on both sides of the hub, out to the trenches.
    for (int s : {-1, 1}) {
      const double y = kWidth / 2 + s * (hubHalf + fc::left_bump::width / 2);
      const auto c = Side(hubX, y, red);
      terrain.AddProfile(c[0], c[1], yaw, fc::left_bump::width, BumpProfile());
    }

    // Trenches: an overhead arm over the opening next to the wall (robots under 22.25 in pass)
    // and a full-height post between the opening and the bump.
    for (int s : {-1, 1}) {
      const double wallY = s < 0 ? 0.0 : kWidth;
      const double openingCenter = wallY - s * fc::left_trench::openingWidth / 2;
      // From the opening to the bump (FieldConstants: the bump ends 12 in past the opening).
      const double postWidth =
          kWidth / 2 - hubHalf - fc::left_bump::width - fc::left_trench::openingWidth;
      const double postCenter = wallY - s * (fc::left_trench::openingWidth + postWidth / 2);
      const auto arm = Side(hubX, openingCenter, red);
      obstacles.AddBox(kTrenchArmPrefix + side, arm[0], arm[1], yaw, fc::left_trench::depth,
                       fc::left_trench::openingWidth, kTrenchClearanceM, fc::left_trench::height);
      const auto post = Side(hubX, postCenter, red);
      obstacles.AddBox(kTrenchPostPrefix + side, post[0], post[1], yaw, fc::left_trench::depth,
                       postWidth, 0, fc::left_trench::height);
    }

    // Tower on the alliance wall.
    {
      const auto c = Side(fc::tower::frontFaceX / 2, fc::kTag31Y, red);
      obstacles.AddBox(kTowerPrefix + side, c[0], c[1], yaw, fc::tower::frontFaceX,
                       fc::tower::width, 0, fc::tower::height);
    }

    // Depot: a 1.125 in frame on the carpet against the alliance wall.
    {
      const auto c = Side(fc::depot::depth / 2, kWidth / 2 + fc::depot::distanceFromCenterY, red);
      terrain.AddBox(c[0], c[1], yaw, fc::depot::depth, fc::depot::width, fc::depot::height);
    }
  }
}

std::vector<std::array<double, 2>> StagedFuel() {
  std::vector<std::array<double, 2>> out;
  constexpr double kPitch = 6.0 * 0.0254;
  // Neutral zone: 12 columns x two blocks of 15 rows (the blocks are 8 in apart).
  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < 30; j++) {
      const double y = 1.799 + j * kPitch + (j >= 15 ? 0.203 - kPitch : 0.0);
      out.push_back({7.4323 + i * kPitch, y});
    }
  }
  // Depots: 4 x 6 each.
  for (bool red : {false, true}) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 6; j++) {
        out.push_back(Side(0.076 + i * kPitch, 5.584 + j * kPitch, red));
      }
    }
  }
  return out;
}

}  // namespace rebuilt
