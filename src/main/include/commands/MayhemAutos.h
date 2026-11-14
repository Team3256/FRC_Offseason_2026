#pragma once
#include <frc/geometry/Pose2d.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>
#include "commands/AutoChooser.h"
#include "commands/MayhemProject.h"

class Superstructure;
class ShotCalculator;
namespace subsystems {
class Drivetrain;
}

/** Mayhem routines from robot.mayhem.json: execution and the disabled field preview. */
namespace suite_autos {
bool IsRed();
frc::Pose2d ToWpi(mayhem::Pose pose);
/** A blue-origin project pose on the current alliance's side. */
mayhem::Pose Resolve(mayhem::Pose pose, bool red);
const mayhem::RoutineInfo* Find(std::string_view chooserName);
std::vector<frc::Pose2d> Preview(const mayhem::RoutineInfo& routine, bool red);
std::unique_ptr<AutoRoutine> Routine(const mayhem::RoutineInfo& routine,
                                     subsystems::Drivetrain& drive, Superstructure& superstructure,
                                     ShotCalculator& shots);
/** Every action, path event and signal the routine uses that the robot does not handle, one per
 * line; empty when all are handled. */
std::string UnboundReport(const mayhem::RoutineInfo& routine, subsystems::Drivetrain& drive,
                          Superstructure& superstructure, ShotCalculator& shots);
}  // namespace suite_autos
