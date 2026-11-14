# Desktop (Linux x86-64) build of the robot program for simulation and unit tests WITHOUT GradleRIO.
#
#   ./tools/desktop/fetch_deps.sh            # once: downloads WPILib/Phoenix 6/PhotonLib natives
#   cmake -S . -B build-desktop -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   cmake --build build-desktop -j
#   ctest --test-dir build-desktop --output-on-failure
#   ./build-desktop/robot-sim                # headless simulation of the full robot program
#
# The robot itself is built and deployed with GradleRIO (./gradlew build / deploy), which uses the
# official WPILib libraries. See README.md.
cmake_minimum_required(VERSION 3.22)
project(FRC3256Robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(DEPS_DIR "${CMAKE_SOURCE_DIR}/build-desktop/deps" CACHE PATH "Output of tools/desktop/fetch_deps.sh")
if(NOT EXISTS "${DEPS_DIR}/lib/libwpilibc.so")
  message(FATAL_ERROR "Run ./tools/desktop/fetch_deps.sh first (expected ${DEPS_DIR}/lib/libwpilibc.so)")
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations
                    -Wno-missing-field-initializers -Wno-psabi)

# ----------------------------------------------------------------------------- WPILib
add_library(wpilib INTERFACE)
target_include_directories(wpilib INTERFACE "${DEPS_DIR}/inc")
target_link_directories(wpilib INTERFACE "${DEPS_DIR}/lib")
target_link_libraries(wpilib INTERFACE wpilibc wpiHal ntcore wpimath wpinet wpiutil)

# ----------------------------------------------------------------------------- robot code
file(GLOB_RECURSE ROBOT_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/main/cpp/*.cpp")
# PhotonLib's desktop libraries need OpenCV/cscore, which are not available outside frcmaven, so the
# PhotonVision IO is only syntax-checked here (target photon-syntax-check). Simulation uses
# VisionIOSim instead; GradleRIO builds the real thing.
list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/VisionIOPhotonVision\\.cpp$")
list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/Main\\.cpp$")

add_library(robotlib STATIC ${ROBOT_SOURCES})
target_include_directories(robotlib PUBLIC src/main/include)
target_compile_definitions(robotlib PUBLIC ROBOT_DESKTOP_BUILD=1)
target_link_libraries(robotlib PUBLIC wpilib CTRE_Phoenix6_WPISim CTRE_PhoenixTools_Sim apriltag
                      pthread)
set_target_properties(robotlib PROPERTIES BUILD_RPATH "${DEPS_DIR}/lib")

add_executable(robot-sim src/main/cpp/Main.cpp)
target_link_libraries(robot-sim PRIVATE robotlib)
set_target_properties(robot-sim PROPERTIES BUILD_RPATH "${DEPS_DIR}/lib")

# Compile (not link) the PhotonVision IO against the real PhotonLib headers.
add_library(photon-syntax-check OBJECT src/main/cpp/subsystems/vision/VisionIOPhotonVision.cpp)
target_include_directories(photon-syntax-check PRIVATE src/main/include)
target_link_libraries(photon-syntax-check PRIVATE wpilib)
set_target_properties(photon-syntax-check PROPERTIES EXCLUDE_FROM_ALL ON)

# ----------------------------------------------------------------------------- tests
find_package(GTest)
if(GTest_FOUND)
  enable_testing()
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/test/cpp/*.cpp")
  add_executable(robot-tests ${TEST_SOURCES})
  target_compile_definitions(robot-tests PRIVATE RUNNING_FRC_TESTS=1)
  target_link_libraries(robot-tests PRIVATE robotlib GTest::gtest)
  set_target_properties(robot-tests PROPERTIES BUILD_RPATH "${DEPS_DIR}/lib")
  include(GoogleTest)
  gtest_discover_tests(robot-tests WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" DISCOVERY_TIMEOUT 120
                       PROPERTIES ENVIRONMENT "ROBOT_DEPLOY_DIR=${CMAKE_SOURCE_DIR}/src/main/deploy")
endif()
