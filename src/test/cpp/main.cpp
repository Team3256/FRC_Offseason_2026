// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include <cstdlib>

#include <gtest/gtest.h>
#include <hal/HALBase.h>

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  // Tests don't write log files unless asked to.
  setenv("ROBOT_LOG_DIR", "", 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
