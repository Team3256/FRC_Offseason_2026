// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace test_util {

/** Finds a repository path (tests may run from the repo root or from a build directory). */
inline std::string RepoPath(const std::string& relative) {
  namespace fs = std::filesystem;
  fs::path dir = fs::current_path();
  for (int i = 0; i < 8; i++) {
    if (fs::exists(dir / relative)) {
      return (dir / relative).string();
    }
    if (!dir.has_parent_path() || dir.parent_path() == dir) {
      break;
    }
    dir = dir.parent_path();
  }
  throw std::runtime_error("Could not find " + relative + " from " + fs::current_path().string());
}

/** Points the trajectory loader at the repository's deploy directory. */
inline void UseRepoDeployDir() {
  setenv("ROBOT_DEPLOY_DIR", RepoPath("src/main/deploy").c_str(), 1);
}

/** A CSV file with a header row; values are accessed by column name. */
class Csv {
 public:
  explicit Csv(const std::string& relativePath) {
    std::ifstream in{RepoPath(relativePath)};
    std::string line;
    std::getline(in, line);
    auto header = Split(line);
    for (size_t i = 0; i < header.size(); i++) {
      m_columns[header[i]] = i;
    }
    while (std::getline(in, line)) {
      if (!line.empty()) {
        m_rows.push_back(Split(line));
      }
    }
  }

  size_t Size() const { return m_rows.size(); }
  const std::string& Str(size_t row, const std::string& col) const {
    return m_rows.at(row).at(Index(col));
  }
  double D(size_t row, const std::string& col) const { return std::stod(Str(row, col)); }
  int I(size_t row, const std::string& col) const { return std::stoi(Str(row, col)); }

 private:
  size_t Index(const std::string& col) const {
    auto it = m_columns.find(col);
    if (it == m_columns.end()) {
      throw std::runtime_error("no column " + col);
    }
    return it->second;
  }
  static std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> out;
    std::string cell;
    std::stringstream ss{line};
    while (std::getline(ss, cell, ',')) {
      out.push_back(cell);
    }
    if (!line.empty() && line.back() == ',') {
      out.emplace_back();
    }
    return out;
  }

  std::map<std::string, size_t> m_columns;
  std::vector<std::vector<std::string>> m_rows;
};

}  // namespace test_util
