#include "risk_mm_runtime/execution.hpp"
#include <mm_model/wheel_kinematics.hpp>
#include <algorithm>
#include <cmath>

namespace risk_mm {
namespace {
double angle_error(double a,double b) {return std::remainder(a-b,2.*M_PI);}
bool stopped(const RobotFeedback &state) {
  if(std::hypot(state.velocity[0],state.velocity[1])>.02 || std::abs(state.velocity[2])>.03)return false;
  for(int i=3;i<9;++i)if(std::abs(state.velocity[i])>.03)return false;
  return true;
}
}
CoupledExecutor::CoupledExecutor(std::shared_ptr<ActuatorPort> actuator,std::shared_ptr<MeshScene> scene,
                               ValidationLimits model_limits,ExecutionLimits limits)
  :actuator_(std::move(actuator)),scene_(std::move(scene)),model_limits_(model_limits),limits_(limits) {}

bool CoupledExecutor::prepare(std::shared_ptr<const Plan> plan,std::string id,std::string &reason) {
  if(!plan || !plan->trajectory || !plan->failure.empty() || plan->mode!="coupled") {
    reason="missing validated coupled polynomial";return false;
  }
  auto state=actuator_->feedback();
  if(!state.valid){reason="execution feedback invalid: "+state.reason;return false;}
  if(state.model_signature!=plan->task.model_signature){reason="execution model identity mismatch";return false;}
  if(!stopped(state)) {
    reason="measured stop required; velocity=";
    for(double value:state.velocity)reason+=std::to_string(value)+",";
    return false;
  }
  for(int i=0;i<9;++i) {
    const double error=i==2?angle_error(state.position[i],plan->task.start[i]):state.position[i]-plan->task.start[i];
    if(std::abs(error)>(i<3?.025:.03)){reason="measured start changed since planning";return false;}
  }
  if(!actuator_->reserve(id,plan->task.model_signature,plan->task.map_revision,reason))return false;
  plan_=std::move(plan);plan_id_=std::move(id);report_={};report_.phase="ALIGNING";
  alignment_steer_=initialSteeringAlignment(*plan_,model_limits_).target;
  queued_.clear();
  prepared_ros_s_=actuator_->rosTime();origin_ros_s_=-1.;settled_=0;
  reason.clear();return true;
}

bool CoupledExecutor::queue(std::shared_ptr<const Plan> next,std::string &reason) {
  const auto previous=queued_.empty()?plan_:queued_.back();
  if(!previous || !next || !next->trajectory || !next->failure.empty() || previous->task.stop_at_goal ||
     next->task.model_signature!=previous->task.model_signature || next->task.map_revision!=previous->task.map_revision) {
    reason="continuation requires matching validated moving-boundary plans";return false;
  }
  const auto end=previous->trajectory->evaluate(previous->trajectory->duration()),start=next->trajectory->evaluate(0.);
  for(int i=0;i<9;++i) {
    const double dp=i==2?angle_error(end.position[i],start.position[i]):end.position[i]-start.position[i];
    if(std::abs(dp)>1e-4 || std::abs(end.velocity[i]-start.velocity[i])>1e-4 ||
       std::abs(end.acceleration[i]-start.acceleration[i])>.002) {reason="continuation is not C2 at the handoff";return false;}
  }
  queued_.push_back(std::move(next));return true;
}

void CoupledExecutor::cancel(const std::string &reason) {
  actuator_->hold(reason);
  queued_.clear();
  if(!plan_)return; // Preserve a completed result when a later task is stopped.
  report_.phase="HOLD";report_.reason=reason;report_.success=false;
  report_.finish_ros_s=actuator_->rosTime();plan_.reset();
}

void CoupledExecutor::tick(bool environment_current) {
  if(!plan_)return;
  try {
    auto state=actuator_->feedback();const double now=actuator_->rosTime();
    if(!environment_current || !state.valid || state.model_signature!=plan_->task.model_signature ||
       state.owner_plan!=plan_id_ || !state.owner_fault.empty() || now<prepared_ros_s_) {
      cancel("environment, model, feedback or command ownership invalidated");return;
    }
    std::string collision;
    if(!scene_->valid(state.position,state.steer,state.wheel_position,collision,state.base_z,state.roll,state.pitch)) {
      cancel("measured robot state rejected: "+collision);return;
    }
    if(std::abs(state.roll)>.12 || std::abs(state.pitch)>.12 || std::abs(state.base_z)>.03) {
      cancel("nonplanar base feedback");return;
    }
    ActuatorCommand command;command.plan_id=plan_id_;command.model_signature=plan_->task.model_signature;
    command.map_revision=plan_->task.map_revision;
    if(origin_ros_s_<0.) {
      if(now-prepared_ros_s_>limits_.alignment_timeout_s){cancel("steering alignment timed out");return;}
      command.steer=alignment_steer_;command.wheel_velocity.fill(0.);
      for(int i=0;i<6;++i)command.arm[i]=plan_->task.start[i+3];
      if(!actuator_->apply(command,collision)){cancel(collision);return;}
      bool aligned=stopped(state);
      for(size_t i=0;i<4;++i)aligned=aligned && std::abs(state.steer[i]-alignment_steer_[i])<.025;
      if(!aligned)return;
      origin_ros_s_=now;report_.phase="EXECUTING";
    }
    while(now-origin_ros_s_>=plan_->trajectory->duration() && !queued_.empty()) {
      origin_ros_s_+=plan_->trajectory->duration();plan_=queued_.front();queued_.pop_front();++report_.segments_completed;
    }
    const double elapsed=now-origin_ros_s_,duration=plan_->trajectory->duration();report_.elapsed_ros_s=elapsed;
    if(elapsed>=duration && !plan_->task.stop_at_goal){cancel("patrol trajectory buffer exhausted");return;}
    const auto desired=plan_->trajectory->evaluate(std::min(elapsed,duration));
    const double dx=desired.position[0]-state.position[0],dy=desired.position[1]-state.position[1];
    const double yaw_error=angle_error(desired.position[2],state.position[2]);
    const double c=std::cos(state.position[2]),s=std::sin(state.position[2]);
    double arm_error=0.;
    for(int i=0;i<6;++i) {
      arm_error=std::max(arm_error,std::abs(desired.position[i+3]-state.position[i+3]));
      command.arm[i]=desired.position[i+3];
    }
    const double base_error=std::hypot(dx,dy);
    report_.max_base_error_m=std::max(report_.max_base_error_m,base_error);
    report_.max_arm_error_rad=std::max(report_.max_arm_error_rad,arm_error);
    if(base_error>limits_.max_base_error || std::abs(yaw_error)>limits_.max_yaw_error || arm_error>limits_.max_arm_error) {
      cancel("measured tracking error exceeded limits");return;
    }
    double velocity=desired.forward_velocity+.8*(c*dx+s*dy);
    double omega=desired.velocity[2]+1.2*yaw_error+1.5*desired.forward_velocity*(-s*dx+c*dy);
    if(elapsed>=duration) {
      report_.phase="SETTLING";
      velocity=std::abs(c*dx+s*dy)<.008?0.:.8*(c*dx+s*dy);
      omega=std::abs(yaw_error)<.015?0.:1.2*yaw_error;
    }
    const double linear_limit=plan_->task.speed_limits[0]>0.?std::min(model_limits_.base_velocity,plan_->task.speed_limits[0]):model_limits_.base_velocity;
    const double angular_limit=plan_->task.speed_limits[1]>0.?std::min(model_limits_.base_yaw_rate,plan_->task.speed_limits[1]):model_limits_.base_yaw_rate;
    velocity=std::clamp(velocity,-linear_limit,linear_limit);
    omega=std::clamp(omega,-angular_limit,angular_limit);
    const auto wheels=mm_model::inverse(velocity,0.,omega,model_limits_.wheel_positions,model_limits_.wheel_radius,
                                        model_limits_.wheel_velocity,state.steer);
    command.steer=wheels.steer;command.wheel_velocity=wheels.speed;
    if(!actuator_->apply(command,collision)){cancel(collision);return;}
    if(!report_.started) {
      bool moved=std::hypot(state.position[0]-plan_->task.start[0],state.position[1]-plan_->task.start[1])>.001;
      for(int i=3;i<9;++i)moved=moved || std::abs(state.position[i]-plan_->task.start[i])>.002;
      if(moved){report_.started=true;report_.start_ros_s=now;}
    }
    if(elapsed>=duration) {
      if(base_error<limits_.position_tolerance && std::abs(yaw_error)<limits_.yaw_tolerance && arm_error<limits_.arm_tolerance && stopped(state))++settled_;
      else settled_=0;
      if(settled_>=3) {
        actuator_->hold("coupled goal reached and measured stop confirmed");
        report_.phase="COMPLETED";report_.completed=true;report_.success=true;report_.finish_ros_s=now;++report_.segments_completed;plan_.reset();
      } else if(elapsed-duration>limits_.settle_timeout_s)cancel("measured goal/stop confirmation timed out");
    }
  } catch(const std::exception &error) {cancel(error.what());}
}
}
