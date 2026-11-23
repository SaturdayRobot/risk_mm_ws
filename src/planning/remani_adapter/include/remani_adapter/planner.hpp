#pragma once
#include <array>
#include <string>
#include <vector>
#include <optimizer/poly_traj_optimizer.hpp>

namespace risk_mm {
// Shared boundary: x,y,yaw,q1..q6; polynomial internals remain in this adapter.
struct Task {
  std::string task_id;
  unsigned long map_revision;
  std::array<double,9> start, goal;
};
struct Point { double time_s; std::array<double,9> position; };
struct Plan {
  Task task;
  std::string mode{"coupled"}, failure;
  std::vector<Point> points;
  double planning_wall_s{0};
};
class Planner {
 public:
  virtual ~Planner() = default;
  virtual Plan plan(const Task&) = 0;
};
class Validator {
 public:
  virtual ~Validator() = default;
  virtual bool validate(const Plan&, std::string& reason) const = 0;
};
class RemaniPlanner final : public Planner {
  remani_planner::PolyTrajOptimizer optimizer_;
  double sample_s_;
 public:
  RemaniPlanner(rclcpp::Node::SharedPtr node, const std::shared_ptr<GridMap>& map,
                const std::shared_ptr<remani_planner::MMConfig>& model, double sample_s=0.02);
  Plan plan(const Task& task) override;
};
} // namespace risk_mm
