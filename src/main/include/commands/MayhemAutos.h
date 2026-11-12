#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandPtr.h>
#include <string_view>
#include <vector>
#include "commands/MayhemProject.h"

class Superstructure;
class ShotCalculator;
namespace subsystems {
class CommandSwerveDrivetrain;
}

/** Mayhem routines from robot.mayhem.json: execution and the disabled field preview. */
namespace suite_autos {
bool IsRed();
frc::Pose2d ToWpi(mayhem::Pose pose);
/** A blue-origin project pose on the current alliance's side. */
mayhem::Pose Resolve(mayhem::Pose pose, bool red);
const mayhem::RoutineInfo* Find(std::string_view chooserName);
std::vector<frc::Pose2d> Preview(const mayhem::RoutineInfo& routine, bool red);
frc2::CommandPtr Command(const mayhem::RoutineInfo& routine,
                         subsystems::CommandSwerveDrivetrain& drive, Superstructure& superstructure,
                         ShotCalculator& shots);
}  // namespace suite_autos
