#pragma once
#include "risk_mm_runtime/execution.hpp"
#include "risk_mm_runtime/task_goals.hpp"
#include "risk_mm_runtime/time_scaling.hpp"
#include <future>

namespace risk_mm {
struct PatrolState {
  bool active{false},finished{false},success{false};
  std::string phase{"IDLE"},reason;
  size_t planned{0},visited{0},total{0},planning_stops{0};
  double last_planning_ms{0};
};
// One validated segment ahead. Planning runs independently of the actuator;
// the executor changes segments at a common p/v/a boundary without a stop.
class Patrol {
  struct Result {
    std::shared_ptr<const Plan> plan;
    ValidationReport validation;
    TimeAdaptation timing;
    bool valid{false},public_validation_run{false},requested_while_moving{false};
    double validation_ms{0};
    std::string reason;
  };
  Planner &planner_;
  CoupledExecutor &executor_;
  std::shared_ptr<ActuatorPort> actuator_;
  std::shared_ptr<MeshScene> scene_;
  ValidationLimits limits_;
  PatrolState state_;
  Task origin_,next_;
  std::vector<TaskGoal> goals_;
  std::vector<Result> results_;
  struct Visit {size_t index;double stamp,error,arm_error,speed;};
  std::vector<Visit> visits_;
  std::future<Result> work_;
  std::string id_,report_file_;
  double through_speed_{.15},yaw_offset_{0.},still_since_{-1.};
  bool still_counted_{false};
  size_t observed_segments_{0};
  void schedule();
  void save();
 public:
  Patrol(Planner &,CoupledExecutor &,std::shared_ptr<ActuatorPort>,std::shared_ptr<MeshScene>,ValidationLimits);
  ~Patrol();
  bool begin(Task origin,std::vector<TaskGoal>,double through_speed,double yaw_offset,
             std::string id,std::string report_file,std::string &reason);
  void tick(bool environment_current);
  void stop(const std::string &reason);
  const PatrolState &state() const {return state_;}
  bool busy() const {return state_.active || work_.valid();}
  std::shared_ptr<const Plan> lastPlan() const {return results_.empty()?nullptr:results_.back().plan;}
};
}
