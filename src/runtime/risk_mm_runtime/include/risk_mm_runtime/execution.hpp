#pragma once
#include "risk_mm_runtime/validation.hpp"
#include <array>
#include <memory>
#include <string>
#include <deque>

namespace risk_mm {
struct RobotFeedback {
  bool valid{false};
  std::string reason,model_signature,owner_plan,owner_fault;
  double stamp_ros_s{0},base_z{0},roll{0},pitch{0};
  std::array<double,9> position{},velocity{};
  std::array<double,4> steer{},wheel_position{},wheel_velocity{};
};
struct ActuatorCommand {
  std::string plan_id,model_signature;
  unsigned long map_revision{0};
  std::array<double,6> arm{};
  std::array<double,4> steer{},wheel_velocity{};
};
class ActuatorPort {
 public:
  virtual ~ActuatorPort()=default;
  virtual double rosTime() const=0;
  virtual RobotFeedback feedback() const=0;
  virtual bool reserve(const std::string &plan_id,const std::string &model_signature,
                       unsigned long map_revision,std::string &reason)=0;
  virtual bool apply(const ActuatorCommand &,std::string &reason)=0;
  virtual void hold(const std::string &reason)=0;
};
struct ExecutionReport {
  std::string phase{"IDLE"},reason;
  bool started{false},completed{false},success{false};
  double elapsed_ros_s{0},start_ros_s{0},finish_ros_s{0};
  double max_base_error_m{0},max_arm_error_rad{0};
  size_t segments_completed{0};
};
struct ExecutionLimits {
  double position_tolerance{.04},yaw_tolerance{.05},arm_tolerance{.03};
  double max_base_error{.10},max_yaw_error{.35},max_arm_error{.08};
  double alignment_timeout_s{8.},settle_timeout_s{5.};
};
class Executor {
 public:
  virtual ~Executor()=default;
  virtual bool prepare(std::shared_ptr<const Plan>,std::string plan_id,std::string &reason)=0;
  virtual void tick(bool environment_current)=0;
  virtual void cancel(const std::string &reason)=0;
  virtual const ExecutionReport &report() const=0;
};
class CoupledExecutor final : public Executor {
  std::shared_ptr<ActuatorPort> actuator_;
  std::shared_ptr<MeshScene> scene_;
  ValidationLimits model_limits_;
  ExecutionLimits limits_;
  std::shared_ptr<const Plan> plan_;
  std::deque<std::shared_ptr<const Plan>> queued_;
  std::string plan_id_;
  ExecutionReport report_;
  double prepared_ros_s_{0},origin_ros_s_{-1};
  int settled_{0};
 public:
  CoupledExecutor(std::shared_ptr<ActuatorPort>,std::shared_ptr<MeshScene>,ValidationLimits,ExecutionLimits);
  bool prepare(std::shared_ptr<const Plan>,std::string plan_id,std::string &reason) override;
  bool queue(std::shared_ptr<const Plan>,std::string &reason);
  size_t queuedPlans() const {return queued_.size();}
  std::shared_ptr<const Plan> currentPlan() const {return plan_;}
  void resetReport() {report_={};}
  void tick(bool environment_current) override;
  void cancel(const std::string &reason) override;
  const ExecutionReport &report() const override {return report_;}
};
}
