// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <units/length.h>

/**
 * Locations of field elements and other reference points, relative to the field coordinate system
 * from the perspective of the blue alliance station.
 *
 * The Java version computed these from deploy/apriltags/welded/2026-official.json at startup. The
 * values below are the same numbers, taken from that file, as compile-time constants (no file I/O
 * or static-initialisation order issues at boot). FieldConstantsTest checks they still match the
 * deployed layout file.
 */
namespace field_constants {

inline constexpr double kInch = 0.0254;
constexpr double Inches(double in) { return in * kInch; }

// From apriltags/welded/2026-official.json
inline constexpr int kAprilTagCount = 32;
inline constexpr double kFieldLength = 16.541;
inline constexpr double kFieldWidth = 8.069;
inline constexpr double kTag26X = 4.0218614;  // alliance hub near face
inline constexpr double kTag4X = 11.3118646;  // opposing hub
inline constexpr double kTag10X = 12.519177399999998;
inline constexpr double kTag31Y = 3.7457125999999996;
inline constexpr double kTag15Y = 4.3235626;
inline constexpr double kTag29Y = 0.6659626;

inline constexpr double kAprilTagWidth = Inches(6.5);
inline constexpr double fieldLength = kFieldLength;
inline constexpr double fieldWidth = kFieldWidth;

namespace hub {
inline constexpr double width = Inches(47.0);
inline constexpr double height = Inches(72.0);  // includes the catcher at the top
inline constexpr double innerWidth = Inches(41.7);
inline constexpr double innerHeight = Inches(56.5);

inline constexpr double kCenterX = kTag26X + width / 2.0;
inline constexpr double kOppCenterX = kTag4X + width / 2.0;

inline const frc::Translation3d topCenterPoint{
    units::meter_t{kCenterX}, units::meter_t{fieldWidth / 2.0}, units::meter_t{height}};
inline const frc::Translation3d innerCenterPoint{
    units::meter_t{kCenterX}, units::meter_t{fieldWidth / 2.0}, units::meter_t{innerHeight}};
inline const frc::Translation2d nearLeftCorner{units::meter_t{kCenterX - width / 2.0},
                                               units::meter_t{fieldWidth / 2.0 + width / 2.0}};
inline const frc::Translation2d nearRightCorner{units::meter_t{kCenterX - width / 2.0},
                                                units::meter_t{fieldWidth / 2.0 - width / 2.0}};
inline const frc::Translation2d farLeftCorner{units::meter_t{kCenterX + width / 2.0},
                                              units::meter_t{fieldWidth / 2.0 + width / 2.0}};
inline const frc::Translation2d farRightCorner{units::meter_t{kCenterX + width / 2.0},
                                               units::meter_t{fieldWidth / 2.0 - width / 2.0}};

inline const frc::Translation3d oppTopCenterPoint{
    units::meter_t{kOppCenterX}, units::meter_t{fieldWidth / 2.0}, units::meter_t{height}};
inline const frc::Translation2d oppNearLeftCorner{units::meter_t{kOppCenterX - width / 2.0},
                                                  units::meter_t{fieldWidth / 2.0 + width / 2.0}};
inline const frc::Translation2d oppNearRightCorner{units::meter_t{kOppCenterX - width / 2.0},
                                                   units::meter_t{fieldWidth / 2.0 - width / 2.0}};
inline const frc::Translation2d oppFarLeftCorner{units::meter_t{kOppCenterX + width / 2.0},
                                                 units::meter_t{fieldWidth / 2.0 + width / 2.0}};
inline const frc::Translation2d oppFarRightCorner{units::meter_t{kOppCenterX + width / 2.0},
                                                  units::meter_t{fieldWidth / 2.0 - width / 2.0}};

/** Blue hub target in 2D (the shot calculator's default target). */
inline const frc::Translation2d kTarget2d{units::meter_t{kCenterX},
                                          units::meter_t{fieldWidth / 2.0}};
/** Red hub target in 2D. */
inline const frc::Translation2d kOppTarget2d{units::meter_t{kOppCenterX},
                                             units::meter_t{fieldWidth / 2.0}};
}  // namespace hub

namespace lines_vertical {
inline constexpr double center = fieldLength / 2.0;
inline constexpr double starting = kTag26X;
inline constexpr double allianceZone = starting;
inline constexpr double hubCenter = kTag26X + hub::width / 2.0;
inline constexpr double neutralZoneNear = center - Inches(120);
inline constexpr double neutralZoneFar = center + Inches(120);
inline constexpr double oppHubCenter = kTag4X + hub::width / 2.0;
inline constexpr double oppAllianceZone = kTag10X;
}  // namespace lines_vertical

namespace left_bump {
inline constexpr double width = Inches(73.0);
inline constexpr double height = Inches(6.513);
inline constexpr double depth = Inches(44.4);
}  // namespace left_bump

namespace right_bump {
inline constexpr double width = Inches(73.0);
inline constexpr double height = Inches(6.513);
inline constexpr double depth = Inches(44.4);
}  // namespace right_bump

namespace lines_horizontal {
inline constexpr double center = fieldWidth / 2.0;
// Right of hub
inline constexpr double rightBumpStart = fieldWidth / 2.0 - hub::width / 2.0;  // hub near right y
inline constexpr double rightBumpEnd = rightBumpStart - right_bump::width;
inline constexpr double rightTrenchOpenStart = rightBumpEnd - Inches(12.0);
inline constexpr double rightTrenchOpenEnd = 0;
// Left of hub
inline constexpr double leftBumpEnd = fieldWidth / 2.0 + hub::width / 2.0;  // hub near left y
inline constexpr double leftBumpStart = leftBumpEnd + left_bump::width;
inline constexpr double leftTrenchOpenEnd = leftBumpStart + Inches(12.0);
inline constexpr double leftTrenchOpenStart = fieldWidth;
}  // namespace lines_horizontal

namespace left_trench {
inline constexpr double width = Inches(65.65);
inline constexpr double depth = Inches(47.0);
inline constexpr double height = Inches(40.25);
inline constexpr double openingWidth = Inches(50.34);
inline constexpr double openingHeight = Inches(22.25);
}  // namespace left_trench

namespace right_trench {
inline constexpr double width = Inches(65.65);
inline constexpr double depth = Inches(47.0);
inline constexpr double height = Inches(40.25);
inline constexpr double openingWidth = Inches(50.34);
inline constexpr double openingHeight = Inches(22.25);
}  // namespace right_trench

namespace tower {
inline constexpr double width = Inches(49.25);
inline constexpr double depth = Inches(45.0);
inline constexpr double height = Inches(78.25);
inline constexpr double innerOpeningWidth = Inches(32.250);
inline constexpr double frontFaceX = Inches(43.51);
inline constexpr double uprightHeight = Inches(72.1);
inline constexpr double lowRungHeight = Inches(27.0);
inline constexpr double midRungHeight = Inches(45.0);
inline constexpr double highRungHeight = Inches(63.0);
inline const frc::Translation2d centerPoint{units::meter_t{frontFaceX}, units::meter_t{kTag31Y}};
inline const frc::Translation2d oppCenterPoint{units::meter_t{fieldLength - frontFaceX},
                                               units::meter_t{kTag15Y}};
}  // namespace tower

namespace depot {
inline constexpr double width = Inches(42.0);
inline constexpr double depth = Inches(27.0);
inline constexpr double height = Inches(1.125);
inline constexpr double distanceFromCenterY = Inches(75.93);
}  // namespace depot

namespace outpost {
inline constexpr double width = Inches(31.8);
inline constexpr double openingDistanceFromFloor = Inches(28.1);
inline constexpr double height = Inches(7.0);
inline const frc::Translation2d centerPoint{0_m, units::meter_t{kTag29Y}};
}  // namespace outpost

/**
 * Choreo's 2026 alliance flip (rotate 180 degrees about the field centre). Used for the corner
 * feed targets; same as ChoreoAllianceFlipUtil.flip(Translation2d) in the Java code.
 */
inline frc::Translation2d FlipRotational(const frc::Translation2d& t) {
  return frc::Translation2d{units::meter_t{fieldLength} - t.X(),
                            units::meter_t{fieldWidth} - t.Y()};
}

}  // namespace field_constants
