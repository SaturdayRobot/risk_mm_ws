#include "risk_mm_runtime/patrol.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <fstream>

namespace risk_mm {
Patrol::Patrol(Planner &planner,CoupledExecutor &executor,std::shared_ptr<ActuatorPort> actuator,
    std::shared_ptr<MeshScene> scene,ValidationLimits limits)
  :planner_(planner),executor_(executor),actuator_(std::move(actuator)),scene_(std::move(scene)),limits_(limits) {}
Patrol::~Patrol(){if(work_.valid())work_.wait();}

bool Patrol::begin(Task origin,std::vector<TaskGoal> goals,double through_speed,double yaw_offset,
    std::string id,std::string report_file,std::string &reason) {
  if(state_.active || work_.valid()){reason="patrol planner busy";return false;}
  if(goals.empty() || !std::isfinite(through_speed) || through_speed<=0.) {reason="patrol needs goals and positive through speed";return false;}
  origin_=std::move(origin);next_=origin_;goals_=std::move(goals);id_=std::move(id);report_file_=std::move(report_file);
  through_speed_=through_speed;yaw_offset_=yaw_offset;results_.clear();visits_.clear();observed_segments_=0;still_since_=-1.;still_counted_=false;
  state_={};state_.active=true;state_.phase="PLANNING";state_.total=goals_.size();
  executor_.resetReport();
  schedule();return true;
}
void Patrol::schedule() {
  const size_t index=state_.planned;
  next_.task_id=id_+"/"+std::to_string(index);
  next_.stop_at_goal=index==goals_.size();next_.goal_tangent_speed=through_speed_;
  if(next_.stop_at_goal)next_.goal=origin_.start;
  else {next_.goal=goals_[index].input;next_.goal[2]=std::remainder(next_.goal[2]-yaw_offset_,2*M_PI);}
  if(index==0) {
    // Inspection is a position region (no image alignment yet). Preserve a
    // nearly straight initial approach instead of amplifying sub-mm odometry
    // yaw error at REMANI's non-singular starting speed.
    const double yaw=origin_.start[2]+yaw_offset_,c=std::cos(yaw),s=std::sin(yaw);
    const double dx=next_.goal[0]-origin_.start[0],dy=next_.goal[1]-origin_.start[1];
    const double along=c*dx+s*dy,lateral=-s*dx+c*dy;
    if(along>0. && std::abs(lateral)<.02 && std::abs(std::remainder(next_.goal[2]-origin_.start[2],2*M_PI))<.02) {
      next_.goal[0]=origin_.start[0]+along*c;next_.goal[1]=origin_.start[1]+along*s;next_.goal[2]=origin_.start[2];
    }
  }
  const Task task=next_;const auto measured=actuator_->feedback();
  const bool moving=std::hypot(measured.velocity[0],measured.velocity[1])>.01;
  work_=std::async(std::launch::async,[this,task,moving]{
    Result result;result.requested_while_moving=moving;
    auto candidate=std::make_shared<Plan>();result.plan=candidate;
    try {*candidate=planner_.plan(task);}catch(const std::exception &error){candidate->failure=error.what();}
    if(!result.plan->failure.empty()){result.reason=result.plan->failure;return result;}
    const auto start=std::chrono::steady_clock::now();
    result.plan=adaptCandidateTime(*candidate,limits_,3.,"through",result.timing,result.reason);
    if(!result.plan) {
      result.plan=candidate;
      result.validation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      return result;
    }
    SampledValidator validator(scene_,limits_,task.model_signature,task.map_revision);
    result.public_validation_run=true;
    result.valid=validator.validate(*result.plan,result.reason);result.validation=validator.report();
    result.validation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    return result;
  });
}
void Patrol::tick(bool environment_current) {
  if(!state_.active) {
    if(work_.valid() && work_.wait_for(std::chrono::seconds(0))==std::future_status::ready)work_.get();
    return;
  }
  if(!environment_current){stop("patrol scene became invalid");return;}
  if(work_.valid() && work_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
    auto result=work_.get();state_.last_planning_ms=result.plan->planning_wall_s*1000.;
    if(!result.valid){results_.push_back(result);stop("patrol planning/validation: "+result.reason);return;}
    std::string reason;
    const bool accepted=state_.planned==0?executor_.prepare(result.plan,id_,reason):executor_.queue(result.plan,reason);
    if(!accepted){results_.push_back(result);stop(reason);return;}
    const auto end=result.plan->trajectory->evaluate(result.plan->trajectory->duration());
    next_=result.plan->task;next_.start=end.position;next_.start_velocity=end.velocity;next_.start_acceleration=end.acceleration;
    // Handoffs are C2. A fresh MINCO horizon uses zero boundary jerk rather
    // than extrapolating the previous segment's time-map derivative.
    next_.start_jerk.fill(0.);next_.moving_start=true;next_.start_steer=result.validation.end_steer;
    next_.start_wheel_position=result.validation.end_wheel_position;
    results_.push_back(std::move(result));++state_.planned;
  }
  executor_.tick(environment_current);
  const auto &execution=executor_.report();state_.phase=execution.phase;
  if(state_.planned==0)state_.phase="PLANNING";
  if(state_.planned>0 && execution.phase=="HOLD"){stop(execution.reason);return;}
  if(execution.segments_completed>observed_segments_) {
    const auto measured=actuator_->feedback();
    for(size_t i=observed_segments_;i<execution.segments_completed && i<goals_.size();++i) {
      const double error=std::hypot(measured.position[0]-goals_[i].input[0],measured.position[1]-goals_[i].input[1]);
      double arm_error=0.;for(int j=3;j<9;++j)arm_error=std::max(arm_error,std::abs(measured.position[j]-goals_[i].input[j]));
      visits_.push_back({i,actuator_->rosTime(),error,arm_error,std::hypot(measured.velocity[0],measured.velocity[1])});
      if(error>.08 || arm_error>.04) {
        stop("measured inspection point was missed");return;
      }
      ++state_.visited;
    }
    observed_segments_=execution.segments_completed;
  }
  if(execution.completed) {
    state_.active=false;state_.finished=true;state_.success=execution.success && state_.visited==goals_.size();
    state_.phase=state_.success?"COMPLETED":"HOLD";state_.reason=execution.reason;save();return;
  }
  const auto motion=actuator_->feedback();
  if(execution.started && std::hypot(motion.velocity[0],motion.velocity[1])>.01 &&
      !work_.valid() && executor_.queuedPlans()==0 && state_.planned<=goals_.size())schedule();
  if(execution.started && execution.phase=="EXECUTING") {
    const auto measured=actuator_->feedback();const double now=actuator_->rosTime();
    if(std::hypot(measured.velocity[0],measured.velocity[1])>.005){still_since_=-1.;still_counted_=false;}
    else if(still_since_<0.)still_since_=now;
    else if(now-still_since_>.2 && !still_counted_){++state_.planning_stops;still_counted_=true;}
  }
}
void Patrol::stop(const std::string &reason) {
  if(!state_.active){executor_.cancel(reason);return;}
  executor_.cancel(reason);state_.active=false;state_.finished=true;state_.success=false;state_.phase="HOLD";state_.reason=reason;save();
}
void Patrol::save() {
  if(report_file_.empty())return;
  YAML::Node report;report["mission_id"]=id_;report["success"]=state_.success;report["phase"]=state_.phase;report["reason"]=state_.reason;
  report["execution_started"]=executor_.report().started;report["execution_completed"]=executor_.report().completed;
  report["inspection_points"]=state_.total;report["visited"]=state_.visited;report["planning_stops"]=state_.planning_stops;
  report["kinematic_validation"]="current Rangermini four-wheel public validator; upstream differential model is not the execution model";
  const auto measured=actuator_->feedback();
  report["return_error_m"]=std::hypot(measured.position[0]-origin_.start[0],measured.position[1]-origin_.start[1]);
  for(double value:origin_.start)report["origin"].push_back(value);
  for(const auto &visit:visits_) {
    YAML::Node point;point["name"]=goals_[visit.index].id;point["ros_time"]=visit.stamp;point["position_error_m"]=visit.error;
    point["arm_error_rad"]=visit.arm_error;point["measured_speed_m_s"]=visit.speed;report["visits"].push_back(point);
  }
  for(size_t i=0;i<results_.size();++i) {
    const auto &result=results_[i];const auto &plan=*result.plan;YAML::Node stage;
    stage["index"]=i;stage["name"]=i<goals_.size()?goals_[i].id:"return_origin";
    stage["planning_ms"]=plan.planning_wall_s*1000.;stage["validation_ms"]=result.validation_ms;
    stage["search_ms"]=plan.search_wall_s*1000.;stage["optimization_ms"]=plan.optimization_wall_s*1000.;
    stage["source_check_ms"]=plan.source_check_wall_s*1000.;stage["budget_hit"]=plan.optimization_budget_hit;
    stage["cost_evaluations"]=plan.cost_evaluations;
    stage["startup_ramp_duration_s"]=plan.start_transition_duration_s;
    stage["startup_ramp_distance_m"]=plan.start_transition_distance_m;
    stage["candidate_generated"]=plan.failure.empty();stage["validation_passed"]=result.valid;
    stage["timing_profile"]=result.timing.profile;stage["time_ratio"]=result.timing.duration_ratio;
    stage["public_validation_run"]=result.public_validation_run;
    stage["first_violation_constraint"]=result.timing.first_violation_constraint;
    stage["first_violation_time_s"]=result.timing.first_violation_time_s;
    stage["first_violation_ratio"]=result.timing.first_violation_ratio;
    stage["best_scale_candidate"]=result.timing.best_scale;
    stage["best_scale_max_constraint"]=result.timing.best_constraint;
    stage["best_scale_max_time_s"]=result.timing.best_time_s;
    stage["best_scale_max_ratio"]=result.timing.best_ratio;
    stage["source_time_knots"]=result.timing.source_knots;stage["execution_time_knots"]=result.timing.time_knots;
    stage["time_scale_knots"]=result.timing.scale_knots;
    for(double value:plan.task.goal)stage["planned_goal"].push_back(value);
    stage["requested_while_moving"]=result.requested_while_moving;stage["reason"]=result.reason;
    if(plan.trajectory)stage["duration_s"]=plan.trajectory->duration();
    if(!result.valid && plan.trajectory)for(const auto &point:plan.points) {
      YAML::Node sample;sample["time_s"]=point.time_s;
      for(double value:point.position)sample["position"].push_back(value);
      stage["unmapped_candidate"].push_back(sample);
    }
    report["segments"].push_back(stage);
  }
  std::ofstream file(report_file_);file<<report<<'\n';
}
}
