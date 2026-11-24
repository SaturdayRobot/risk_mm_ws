#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <optimizer/poly_traj_optimizer.hpp>

namespace risk_mm {
// Shared boundary: x,y,yaw,q1..q6; polynomial internals remain in this adapter.
struct Task {
  std::string task_id;
  unsigned long map_revision;
  std::array<double,9> start, goal;
  std::string model_signature;
  std::array<double,4> start_steer{}, start_wheel_position{};
  std::array<double,3> speed_limits{};
  std::array<double,9> start_velocity{},start_acceleration{},start_jerk{};
  double goal_tangent_speed{.02};
  bool moving_start{false},stop_at_goal{true},require_start_boundary{false};
};
struct Point { double time_s; std::array<double,9> position; };
struct TrajectoryState {
  std::array<double,9> position{}, velocity{}, acceleration{},jerk{};
  double forward_velocity{0};
};
class Trajectory {
 public:
  virtual ~Trajectory() = default;
  virtual double duration() const = 0;
  virtual TrajectoryState evaluate(double time_s) const = 0;
  virtual std::vector<double> boundaries() const = 0;
};
struct Plan {
  Task task;
  std::string mode{"coupled"}, failure;
  std::string source_validation{"not_generated"};
  std::vector<Point> points;
  // Populated only by explicit offline diagnostics; never consumed by execution.
  std::vector<std::array<double,9>> diagnostic_search_path;
  std::vector<Point> diagnostic_minco_seed;
  std::vector<Point> diagnostic_optimized_candidate;
  double planning_wall_s{0};
  double search_wall_s{0},optimization_wall_s{0},source_check_wall_s{0};
  double start_transition_duration_s{0},start_transition_distance_m{0};
  int cost_evaluations{0};
  bool optimization_budget_hit{false},warm_started{false};
  std::shared_ptr<const Trajectory> trajectory;
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
  double tangent_speed_;
  double planning_yaw_in_base_{0.};
  std::uint32_t search_seed_{0};
  bool fixed_search_seed_{false},forward_only_{false};
  bool capture_diagnostics_{false};
  Plan solve(const Task&,std::shared_ptr<const Trajectory>,double);
 public:
  RemaniPlanner(rclcpp::Node::SharedPtr node, const std::shared_ptr<GridMap>& map,
                const std::shared_ptr<remani_planner::MMConfig>& model, double sample_s=0.02);
  Plan plan(const Task& task) override;
  Plan replan(const Task&,std::shared_ptr<const Trajectory> reference,double reference_time);
  void setComputeBudgetMs(double value) {optimizer_.setComputeBudgetMs(value);}
  void setForwardOnly(bool enabled,double radius=.55) {forward_only_=enabled;optimizer_.kino_a_star_->setForwardOnly(enabled,radius);}
  void setExternalKinematicValidation(bool value) {optimizer_.setExternalKinematicValidation(value);}
  void setCaptureDiagnostics(bool enabled) {capture_diagnostics_=enabled;}
};
} // namespace risk_mm
