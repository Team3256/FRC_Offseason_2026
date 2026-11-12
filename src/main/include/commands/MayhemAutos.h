#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandPtr.h>
#include <memory>
#include <string_view>
#include <vector>
#include "commands/MayhemProject.h"

class Superstructure;
class ShotCalculator;
namespace subsystems {
class CommandSwerveDrivetrain;
}

/** Mayhem Suite graphs are the source for execution AND disabled previews. */
namespace suite_autos {
bool IsRed();
frc::Pose2d ToWpi(mayhem::Pose pose);
mayhem::Pose Resolve(mayhem::Pose pose, bool red);
const robot_mayhem::AutoDefinition* Find(std::string_view chooserName);
std::vector<frc::Pose2d> Preview(const robot_mayhem::AutoDefinition& definition, bool red);
frc2::CommandPtr Command(const robot_mayhem::AutoDefinition& definition,
                         subsystems::CommandSwerveDrivetrain& drive, Superstructure& superstructure,
                         ShotCalculator& shots);
}  // namespace suite_autos
