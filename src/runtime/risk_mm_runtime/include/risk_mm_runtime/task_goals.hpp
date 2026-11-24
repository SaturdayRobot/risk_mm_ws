#pragma once
#include <array>
#include <string>
#include <vector>

namespace risk_mm {
// Explicit operator task inputs: world x/y, base_control heading, joint1..6.
// These are never consulted by a mode policy or an ARM_FIRST stow planner.
struct TaskGoal {
  std::string id, label, description;
  std::array<double,9> input{};
};
std::vector<TaskGoal> loadTaskGoals(const std::string &file,
    const std::string &world,const std::string &world_sha256);
}
