#include "risk_mm_runtime/task_goals.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <set>
#include <stdexcept>

namespace risk_mm {
std::vector<TaskGoal> loadTaskGoals(const std::string &file,
    const std::string &world,const std::string &world_sha256) {
  const auto config=YAML::LoadFile(file);
  if(config["schema_version"].as<int>()!=1 || config["world"].as<std::string>()!=world ||
     config["world_sha256"].as<std::string>()!=world_sha256 ||
     config["position_frame"].as<std::string>()!="world" ||
     config["heading_frame"].as<std::string>()!="base_control" ||
     config["angle_unit"].as<std::string>()!="rad")
    throw std::runtime_error("task presets do not match the selected world/coordinate convention");
  const auto entries=config["goals"];
  if(!entries.IsSequence() || entries.size()==0 || entries.size()>20)
    throw std::runtime_error("task presets require 1..20 explicit goals");
  std::vector<TaskGoal> result;std::set<std::string> ids;
  for(const auto &entry:entries) {
    TaskGoal goal;
    goal.id=entry["id"].as<std::string>();goal.label=entry["label"].as<std::string>();
    goal.description=entry["description"].as<std::string>();
    if(goal.id.empty() || goal.label.empty() || goal.description.empty() || !ids.insert(goal.id).second)
      throw std::runtime_error("task presets require distinct IDs and nonempty labels/descriptions");
    const auto base=entry["base"].as<std::vector<double>>(),arm=entry["arm"].as<std::vector<double>>();
    if(base.size()!=3 || arm.size()!=6)throw std::runtime_error("task preset requires base[3] and arm[6]");
    for(size_t i=0;i<3;++i)goal.input[i]=base[i];
    for(size_t i=0;i<6;++i)goal.input[i+3]=arm[i];
    for(double value:goal.input)if(!std::isfinite(value))throw std::runtime_error("nonfinite task preset");
    goal.input[2]=std::remainder(goal.input[2],2.*M_PI);
    result.push_back(std::move(goal));
  }
  return result;
}
}
