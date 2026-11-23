#pragma once
#include <string>

namespace risk_mm {
// Build a Gazebo-only description; never alter the source planning URDF.
std::string prepare_simulation_model();
}
