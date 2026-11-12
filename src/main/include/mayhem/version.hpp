#pragma once
// Mayhem C++ library version. CMake, the MayhemCpp vendordep and `mayhem.mjs vendor`
// read MAYHEM_CPP_VERSION from this line.
#define MAYHEM_CPP_VERSION "0.2.0"
#include <string_view>
namespace mayhem {
inline constexpr std::string_view kCppVersion = MAYHEM_CPP_VERSION;
}  // namespace mayhem
