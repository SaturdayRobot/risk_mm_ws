#include "risk_mm_runtime/runtime.hpp"
#include "risk_mm_runtime/time_scaling.hpp"
#include "risk_mm_runtime/patrol.hpp"
#include <risk_mm_interfaces/msg/coupled_status.hpp>
#include <risk_mm_interfaces/srv/plan_coupled.hpp>
#include <risk_mm_interfaces/srv/execute_plan.hpp>
#include <risk_mm_interfaces/srv/start_patrol.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <sstream>
#include <iomanip>
#include <std_msgs/msg/string.hpp>

namespace risk_mm {
class CoupledRuntime::Implementation {
  using PlanService=risk_mm_interfaces::srv::PlanCoupled;
  using ExecuteService=risk_mm_interfaces::srv::ExecutePlan;
  using PatrolService=risk_mm_interfaces::srv::StartPatrol;
  using Status=risk_mm_interfaces::msg::CoupledStatus;
  struct WorkResult {
    std::shared_ptr<const Plan> plan;
    bool valid{false};
    std::string reason;
    ValidationReport validation;
    std::shared_ptr<const Plan> raw_plan;
    bool raw_valid{false};double time_scale{1.};std::string raw_reason;
    TimeAdaptation timing;
  };
  CoupledRuntime &node_;
  rclcpp::Node::SharedPtr backend_;
  std::shared_ptr<ActuatorPort> actuator_;
  std::unique_ptr<WorldScene> world_;
  std::shared_ptr<GridMap> map_;
  std::shared_ptr<remani_planner::MMConfig> model_;
  std::unique_ptr<RemaniPlanner> planner_;
  std::shared_ptr<MeshScene> mesh_;
  std::unique_ptr<SampledValidator> validator_;
  std::unique_ptr<CoupledExecutor> executor_;
  std::unique_ptr<Patrol> patrol_;
  bool patrol_selected_{false};
  std::string patrol_file_,world_name_;
  ValidationLimits validation_limits_;
  bool timing_enabled_{false};double max_time_scale_{1.};
  std::string timing_profile_{"uniform"};
  std::string signature_,report_directory_,scene_reason_,plan_id_,phase_{"IDLE"},reason_,actual_mode_;
  unsigned long revision_{0},plan_revision_{0},sequence_{0};
  unsigned long invalidation_sequence_{0};
  bool scene_valid_{false},scene_seen_{false},map_initialized_{false};
  bool candidate_{false},validated_{false},ready_{false},result_saved_{false};
  double scene_time_{-1},last_tick_{-1},last_status_{-1},planning_wall_{0},validation_wall_{0};
  std::shared_ptr<const Plan> plan_;
  std::future<WorkResult> work_;
  std::string work_id_;
  std::string execution_id_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr patrol_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_pub_,validated_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_odom_pub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr scene_sub_;
  rclcpp::Service<PlanService>::SharedPtr plan_service_;
  rclcpp::Service<ExecuteService>::SharedPtr execute_service_;
  rclcpp::Service<PatrolService>::SharedPtr patrol_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  template<typename T> T parameter(const std::string &name,T fallback) {
    node_.get_parameter_or(name,fallback,fallback);return fallback;
  }
  bool environmentCurrent() const {
    if(!actuator_)return false;
    const double delta=actuator_->rosTime()-scene_time_;
    return scene_valid_ && delta>=0. && delta<=.5;
  }
  void save(const std::string &name,const YAML::Node &data) {
    if(plan_id_.empty())return;
    const auto directory=std::filesystem::path(report_directory_)/"coupled"/plan_id_;
    std::filesystem::create_directories(directory);
    std::ofstream output(directory/name);output<<data<<'\n';
  }
  void path(const Plan &plan,const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher) {
    nav_msgs::msg::Path message;message.header.frame_id="world";message.header.stamp=node_.now();
    for(const auto &point:plan.points) {
      geometry_msgs::msg::PoseStamped pose;pose.header=message.header;
      pose.pose.position.x=point.position[0];pose.pose.position.y=point.position[1];
      pose.pose.orientation.z=std::sin(point.position[2]/2);pose.pose.orientation.w=std::cos(point.position[2]/2);
      message.poses.push_back(pose);
    }publisher->publish(message);
  }
  void invalidate(const std::string &why) {
    const bool active=phase_=="ALIGNING" || phase_=="EXECUTING" || phase_=="SETTLING";
    ready_=false;reason_=why;phase_="HOLD";
    if(patrol_selected_ && patrol_)patrol_->stop(why);
    else if(executor_)executor_->cancel(why);
    else if(actuator_)actuator_->hold(why);
    if(active)saveExecution();
    if(!plan_id_.empty()) {YAML::Node event;event["reason"]=why;event["plan_id"]=plan_id_;event["ros_s"]=actuator_->rosTime();save("invalidated_"+std::to_string(++invalidation_sequence_)+".yaml",event);}
  }
  void saveExecution() {
    const auto &execution=executor_->report();
    YAML::Node report;report["plan_id"]=plan_id_;report["model_signature"]=signature_;report["map_revision"]=plan_revision_;
    report["execution_started"]=execution.started;report["execution_completed"]=execution.completed;report["success"]=execution.success;
    report["actual_mode"]=execution.started?"coupled":"";report["reason"]=execution.reason;report["start_ros_s"]=execution.start_ros_s;report["finish_ros_s"]=execution.finish_ros_s;
    report["max_base_error_m"]=execution.max_base_error_m;report["max_arm_error_rad"]=execution.max_arm_error_rad;
    for(double value:actuator_->feedback().position)report["measured_final_state"].push_back(value);
    save("execution.yaml",report);result_saved_=true;
  }
  void requestPlan(const PlanService::Request &request,PlanService::Response &response) {
    response.accepted=false;
    if(!map_initialized_ || !environmentCurrent()){response.reason="complete current Gazebo scene required: "+scene_reason_;return;}
    if(work_.valid() || (patrol_ && patrol_->busy()) || phase_=="EXECUTING" || phase_=="ALIGNING" || phase_=="SETTLING") {response.reason="planner or execution is busy";return;}
    if(request.task_id.empty() || request.model_signature!=signature_ || request.map_revision!=revision_) {response.reason="task/model/map identity mismatch";return;}
    const auto state=actuator_->feedback();
    if(!state.valid){response.reason=state.reason;return;}
    for(double v:request.goal)if(!std::isfinite(v)){response.reason="nonfinite coupled goal";return;}
    Task task;task.task_id=request.task_id;task.model_signature=signature_;task.map_revision=revision_;
    task.start=state.position;task.goal=request.goal;task.start_steer=state.steer;task.start_wheel_position=state.wheel_position;
    auto requested_limits=validation_limits_;
    std::array<double*,3> speed{&requested_limits.base_velocity,&requested_limits.base_yaw_rate,&requested_limits.arm_velocity};
    for(int i=0;i<3;++i) {
      const double value=request.speed_limits[i];
      if(!std::isfinite(value) || value<0. || value>*speed[i]){response.reason="requested speed exceeds configured limits";return;}
      if(value>0.)*speed[i]=value;
      task.speed_limits[i]=*speed[i];
    }
    std::string invalid;
    if(!mesh_->valid(task.start,state.steer,state.wheel_position,invalid,state.base_z,state.roll,state.pitch)) {response.reason="measured start: "+invalid;return;}
    if(!mesh_->valid(task.goal,state.steer,state.wheel_position,invalid)) {response.reason="goal: "+invalid;return;}
    if(!state.owner_plan.empty())actuator_->hold("replace prepared plan");
    const std::string id="coupled_"+std::to_string(++sequence_);
    patrol_selected_=false;planner_->setComputeBudgetMs(parameter("optimization.solve_budget_ms",0.));planner_->setForwardOnly(false);
    planner_->setExternalKinematicValidation(false);
    if(!actuator_->reserve(id,signature_,revision_,invalid)){response.reason=invalid;return;}
    plan_id_=id;work_id_=id;plan_revision_=revision_;phase_="PLANNING";reason_.clear();plan_.reset();
    execution_id_.clear();
    candidate_=validated_=ready_=result_saved_=false;planning_wall_=validation_wall_=0.;
    validator_=std::make_unique<SampledValidator>(mesh_,requested_limits,signature_,revision_);
    work_=std::async(std::launch::async,[this,task,requested_limits] {
      WorkResult result;
      try {
        result.plan=std::make_shared<const Plan>(planner_->plan(task));
        if(result.plan->failure.empty()) {
          const auto checking=std::chrono::steady_clock::now();
          result.valid=validator_->validate(*result.plan,result.reason);result.validation=validator_->report();
          result.raw_plan=result.plan;result.raw_valid=result.valid;result.raw_reason=result.reason;
          if(timing_enabled_ && ((result.valid && timing_profile_=="local") || (!result.valid && timingViolation(result.reason)))) {
            std::string adaptation;
            auto adjusted=adaptCandidateTime(*result.plan,requested_limits,max_time_scale_,timing_profile_,result.timing,adaptation);
            result.time_scale=result.timing.duration_ratio;
            if(adjusted) {
              result.plan=std::move(adjusted);result.valid=validator_->validate(*result.plan,result.reason);
              result.validation=validator_->report();result.reason=adaptation+"; "+result.reason;
            }else {result.valid=false;result.reason=adaptation;}
          }
          result.validation.wall_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-checking).count();
        } else result.reason=result.plan->failure;
      } catch(const std::exception &error){result.reason=error.what();}
      return result;
    });
    response.accepted=true;response.plan_id=id;response.reason="coupled request accepted; candidate generation is pending";
  }
  void requestPatrol(const PatrolService::Request &request,PatrolService::Response &response) {
    response.accepted=false;
    if(work_.valid() || (patrol_ && patrol_->busy()) || phase_=="EXECUTING" || phase_=="ALIGNING" || phase_=="SETTLING") {response.reason="planner or execution busy";return;}
    if(!map_initialized_ || !environmentCurrent() || request.model_signature!=signature_ || request.map_revision!=revision_) {
      response.reason="current complete scene and matching model/map required";return;
    }
    const auto measured=actuator_->feedback();
    if(!measured.valid){response.reason=measured.reason;return;}
    for(double value:measured.velocity)if(std::abs(value)>.025){response.reason="initial measured stop required";return;}
    Task origin;origin.task_id=request.task_id;origin.model_signature=signature_;origin.map_revision=revision_;
    origin.start=measured.position;origin.goal=measured.position;origin.start_steer=measured.steer;origin.start_wheel_position=measured.wheel_position;
    auto limits=validation_limits_;
    std::array<double*,3> speed{&limits.base_velocity,&limits.base_yaw_rate,&limits.arm_velocity};
    for(int i=0;i<3;++i) {
      const double value=request.speed_limits[i];
      if(!std::isfinite(value) || value<0. || value>*speed[i]){response.reason="requested speed exceeds configured limits";return;}
      if(value>0.)*speed[i]=value;
      origin.speed_limits[i]=*speed[i];
    }
    const auto config=YAML::LoadFile(patrol_file_);
    auto goals=loadTaskGoals(patrol_file_,world_name_,world_->signature());
    const double budget=config["optimizer_budget_ms"].as<double>(),through=config["through_speed_m_s"].as<double>();
    if(!std::isfinite(budget) || budget<=0. || through>limits.base_velocity){response.reason="patrol timing configuration invalid";return;}
    const std::string id="patrol_"+std::to_string(++sequence_);
    actuator_->hold("prepare autonomous patrol");
    if(!actuator_->reserve(id,signature_,revision_,response.reason))return;
    planner_->setComputeBudgetMs(budget);
    planner_->setForwardOnly(config["forward_only"].as<bool>(),config["min_turning_radius_m"].as<double>());
    planner_->setExternalKinematicValidation(true);
    patrol_=std::make_unique<Patrol>(*planner_,*executor_,actuator_,mesh_,limits);
    const auto directory=std::filesystem::path(report_directory_)/"patrol"/id;std::filesystem::create_directories(directory);
    std::filesystem::copy_file(patrol_file_,directory/"inputs.yaml",std::filesystem::copy_options::overwrite_existing);
    const double offset=parameter("mm.planning_frame_yaw_in_base_rad",0.);
    if(!patrol_->begin(origin,std::move(goals),through,offset,id,(directory/"mission.yaml").string(),response.reason))return;
    patrol_selected_=true;plan_id_=execution_id_=id;plan_revision_=revision_;phase_="PLANNING";reason_.clear();
    candidate_=validated_=ready_=result_saved_=false;planning_wall_=validation_wall_=0.;plan_.reset();
    response.accepted=true;response.mission_id=id;response.reason="autonomous coupled patrol started; future segments will be planned in motion";
  }
  void requestExecution(const ExecuteService::Request &request,ExecuteService::Response &response) {
    response.accepted=false;
    if(!ready_ || !plan_ || phase_!="READY" || !environmentCurrent() || revision_!=plan_revision_ ||
       request.plan_id!=plan_id_ || request.model_signature!=signature_ || request.map_revision!=plan_revision_) {
      response.reason="fresh matching validated plan required";return;
    }
    if(!executor_->prepare(plan_,plan_id_,response.reason)) {invalidate(response.reason);return;}
    ready_=false;phase_="ALIGNING";result_saved_=false;response.accepted=true;
    execution_id_=plan_id_;
    response.reason="validated plan accepted; waiting for measured alignment and motion";
  }
  void completedWork() {
    if(!work_.valid() || work_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
    auto result=work_.get();
    if(work_id_!=plan_id_ || phase_!="PLANNING")return;
    plan_=result.plan;
    candidate_=plan_ && plan_->failure.empty() && !plan_->points.empty();
    validated_=result.valid;planning_wall_=plan_?plan_->planning_wall_s:0.;validation_wall_=result.validation.wall_s;
    reason_=result.reason;
    YAML::Node report;report["plan_id"]=plan_id_;report["model_signature"]=signature_;report["map_revision"]=plan_revision_;
    report["candidate_generated"]=candidate_;report["validation_passed"]=validated_;report["reason"]=reason_;
    report["source_validation"]=plan_?plan_->source_validation:"no_candidate";
    if(plan_) {
      report["search_wall_s"]=plan_->search_wall_s;report["optimization_wall_s"]=plan_->optimization_wall_s;
      report["source_check_wall_s"]=plan_->source_check_wall_s;report["cost_evaluations"]=plan_->cost_evaluations;
    }
    report["raw_validation_passed"]=result.raw_valid;report["raw_validation_reason"]=result.raw_reason;report["time_scale"]=result.time_scale;
    report["timing_profile"]=result.timing.profile;report["maximum_local_time_scale"]=result.timing.maximum_scale;
    report["source_time_knots"]=result.timing.source_knots;report["execution_time_knots"]=result.timing.time_knots;
    report["local_time_scale_knots"]=result.timing.scale_knots;
    if(plan_)for(double speed:plan_->task.speed_limits)report["requested_speed_limits"].push_back(speed);
    if(result.raw_plan) {
      report["raw_duration_s"]=result.raw_plan->trajectory->duration();
      for(const auto &point:result.raw_plan->points) {
        YAML::Node sample;sample["time_s"]=point.time_s;
        for(double value:point.position)sample["position"].push_back(value);
        report["raw_candidate"].push_back(sample);
      }
    }
    report["planning_wall_s"]=planning_wall_;report["validation_wall_s"]=validation_wall_;
    report["validation_samples"]=result.validation.samples;report["validation_scope"]="sampled full URDF meshes and four-wheel limits; no continuous-safety claim";
    if(candidate_) {
      path(*plan_,candidate_pub_);
      for(const auto &point:plan_->points) {
        YAML::Node sample;sample["time_s"]=point.time_s;
        for(double value:point.position)sample["position"].push_back(value);
        report["candidate"].push_back(sample);
      }
      report["duration_s"]=plan_->trajectory->duration();
    }
    save("planning.yaml",report);
    const auto state=actuator_->feedback();
    bool start_current=state.valid;
    if(plan_)for(int i=0;i<9;++i) {
      const double error=i==2?std::remainder(state.position[i]-plan_->task.start[i],2*M_PI):state.position[i]-plan_->task.start[i];
      start_current=start_current && std::abs(error)<(i<3?.025:.03);
    }
    ready_=candidate_ && validated_ && start_current && environmentCurrent() && revision_==plan_revision_;
    if(ready_){phase_="READY";path(*plan_,validated_pub_);}
    else {phase_="REJECTED";actuator_->hold(reason_);if(validated_)reason_="measured start or scene changed while planning";}
  }
  void publishStatus(double now) {
    if(now-last_status_<.1)return;
    last_status_=now;
    const auto measured=actuator_->feedback();const auto &execution=executor_->report();
    Status status;status.header.frame_id="world";status.header.stamp=node_.now();
    status.model_signature=signature_;status.map_revision=revision_;status.plan_map_revision=plan_revision_;status.plan_id=plan_id_;
    status.phase=phase_;status.map_ready=map_initialized_ && environmentCurrent();status.feedback_ready=measured.valid;
    status.candidate_generated=candidate_;status.validation_passed=validated_;status.ready_to_execute=ready_;
    status.execution_started=execution_id_==plan_id_ && execution.started;
    status.execution_completed=execution_id_==plan_id_ && execution.completed;
    status.execution_success=execution_id_==plan_id_ && execution.success;
    status.actual_mode=actual_mode_;status.reason=status.map_ready?reason_:(scene_reason_.empty()?"waiting for fresh complete Gazebo scene":scene_reason_);
    status.planning_wall_s=planning_wall_;status.validation_wall_s=validation_wall_;
    status.duration_s=plan_ && plan_->trajectory?plan_->trajectory->duration():0.;status.elapsed_ros_s=execution_id_==plan_id_?execution.elapsed_ros_s:0.;
    status.state=measured.position;if(plan_)status.goal=plan_->task.goal;
    status.candidate_samples=plan_?plan_->points.size():0;status_pub_->publish(status);
    if(patrol_selected_) {
      const auto &state=patrol_->state();std::ostringstream json;
      json<<"{\"mission_id\":"<<std::quoted(plan_id_)<<",\"phase\":"<<std::quoted(state.phase)
          <<",\"visited\":"<<state.visited<<",\"total\":"<<state.total<<",\"planned\":"<<state.planned
          <<",\"planning_stops\":"<<state.planning_stops<<",\"planning_ms\":"<<state.last_planning_ms
          <<",\"returned\":"<<(state.success?"true":"false")<<"}";
      std_msgs::msg::String message;message.data=json.str();patrol_pub_->publish(message);
    }
  }
  void tick() {
    const double now=actuator_->rosTime();
    if(last_tick_>=0. && now<last_tick_) {scene_valid_=false;scene_time_=-1.;++revision_;invalidate("ROS time reset");last_status_=-1.;}
    last_tick_=now;
    if(!map_initialized_ && environmentCurrent()) {
      const auto state=actuator_->feedback();
      if(state.valid) {
        nav_msgs::msg::Odometry odom;odom.header.frame_id="world";odom.header.stamp=node_.now();odom.pose.pose.orientation.w=1.;
        map_odom_pub_->publish(odom);
        if(map_->odomValid()) {
          std_msgs::msg::Header header;header.frame_id="world";header.stamp=node_.now();
          if(!map_->replaceCompleteSnapshot(world_->cloud(map_->getResolution(),header)))throw std::runtime_error("complete scene snapshot rejected");
          map_initialized_=true;
        }
      }
    }
    if(patrol_selected_) {
      const auto planned=patrol_->state().planned;
      patrol_->tick(environmentCurrent() && revision_==plan_revision_);
      const auto &state=patrol_->state();phase_=state.phase;reason_=state.reason;
      if(auto current=executor_->currentPlan())plan_=current;
      else if(auto last=patrol_->lastPlan())plan_=last;
      candidate_=validated_=state.planned>0;planning_wall_=state.last_planning_ms/1000.;
      if(executor_->report().started)actual_mode_="coupled";
      if(state.planned!=planned && patrol_->lastPlan())path(*patrol_->lastPlan(),validated_pub_);
      if(state.finished && !result_saved_)saveExecution();
      publishStatus(now);return;
    }
    completedWork();
    if((ready_ || phase_=="PLANNING") && !environmentCurrent())invalidate("scene input expired or changed");
    if(ready_ && plan_) {
      const auto measured=actuator_->feedback();bool valid=measured.valid;
      for(int i=0;i<9;++i) {
        const double error=i==2?std::remainder(measured.position[i]-plan_->task.start[i],2*M_PI):measured.position[i]-plan_->task.start[i];
        valid=valid && std::abs(error)<(i<3?.025:.03);
      }
      if(!valid)invalidate("measured start changed after validation");
    }
    if(phase_=="ALIGNING" || phase_=="EXECUTING" || phase_=="SETTLING") {
      executor_->tick(environmentCurrent() && revision_==plan_revision_);
      const auto &execution=executor_->report();phase_=execution.phase;reason_=execution.reason;
      if(execution.started)actual_mode_="coupled";
      if((execution.completed || phase_=="HOLD") && !result_saved_) {
        saveExecution();
      }
    }
    publishStatus(now);
  }
 public:
  explicit Implementation(CoupledRuntime &node,const rclcpp::NodeOptions &options):node_(node) {
    node_.get_parameter("model_signature",signature_);node_.get_parameter("report_directory",report_directory_);
    if(signature_.rfind("rangermini_xarm6:",0)!=0)throw std::runtime_error("current model signature required");
    backend_=std::make_shared<rclcpp::Node>("coupled_backend",options);
    std::string world,metadata;node_.get_parameter("world_file",world);node_.get_parameter("world_metadata_file",metadata);
    world_=std::make_unique<WorldScene>(world,metadata);revision_=world_->revision();
    world_name_=std::filesystem::path(world).stem().string();node_.get_parameter("patrol_file",patrol_file_);
    std::vector<double> positions;node_.get_parameter("controller.wheel_positions",positions);
    if(positions.size()!=8)throw std::runtime_error("current four-wheel geometry required");
    std::copy(positions.begin(),positions.end(),validation_limits_.wheel_positions.begin());
    validation_limits_.arm_velocity=parameter("validation.arm_velocity",.35);
    validation_limits_.arm_acceleration=parameter("validation.arm_acceleration",.7);
    validation_limits_.wheel_velocity=parameter("validation.wheel_velocity",10.);
    validation_limits_.wheel_acceleration=parameter("validation.wheel_acceleration",5.);
    validation_limits_.steer_rate=parameter("validation.steer_rate",M_PI_2);
    validation_limits_.base_velocity=parameter("validation.base_velocity",1.);
    validation_limits_.base_yaw_rate=parameter("validation.base_yaw_rate",1.2);
    validation_limits_.collision_padding=parameter("validation.collision_padding",.01);
    timing_enabled_=parameter("coupled_time_scaling.enabled",false);
    max_time_scale_=parameter("coupled_time_scaling.max_scale",1.);
    timing_profile_=parameter("coupled_time_scaling.profile",std::string("uniform"));
    if(timing_profile_!="uniform" && timing_profile_!="local")throw std::runtime_error("unknown explicit time profile");
  }
  ~Implementation(){if(work_.valid())work_.wait();}
  const std::string &signature() const {return signature_;}
  rclcpp::Node::SharedPtr backend() const {return backend_;}
  void initialize(std::shared_ptr<ActuatorPort> actuator) {
    actuator_=std::move(actuator);
    std::string urdf,srdf;node_.get_parameter("robot_urdf_file",urdf);node_.get_parameter("robot_srdf_file",srdf);
    std::string expected_urdf;node_.get_parameter("robot_urdf_sha256",expected_urdf);
    if(expected_urdf.empty() || content_sha256(urdf)!=expected_urdf)throw std::runtime_error("robot URDF identity changed after launch configuration");
    const int mesh_workers=parameter("validation.mesh_workers",4);
    if(mesh_workers<1 || mesh_workers>4)throw std::runtime_error("mesh workers must be in 1..4");
    mesh_=std::make_shared<MeshScene>(*world_,urdf,srdf,validation_limits_.collision_padding,mesh_workers);
    validator_=std::make_unique<SampledValidator>(mesh_,validation_limits_,signature_,revision_);
    map_=std::make_shared<GridMap>();map_->initMap(backend_);
    model_=std::make_shared<remani_planner::MMConfig>();model_->setParam(backend_,map_);
    planner_=std::make_unique<RemaniPlanner>(backend_,map_,model_);
    // Batch validation must not monopolize the mutable MoveIt collision state
    // used by the actuator tick. Both contexts use the same frozen geometry.
    auto execution_mesh=std::make_shared<MeshScene>(*world_,urdf,srdf,validation_limits_.collision_padding);
    executor_=std::make_unique<CoupledExecutor>(actuator_,std::move(execution_mesh),validation_limits_,ExecutionLimits{});
    map_odom_pub_=node_.create_publisher<nav_msgs::msg::Odometry>("grid_map/odom",10);
    status_pub_=node_.create_publisher<Status>("/coupled/status",10);
    patrol_pub_=node_.create_publisher<std_msgs::msg::String>("/coupled/patrol_status",10);
    candidate_pub_=node_.create_publisher<nav_msgs::msg::Path>("/coupled/candidate_path",rclcpp::QoS(1).transient_local());
    validated_pub_=node_.create_publisher<nav_msgs::msg::Path>("/coupled/validated_path",rclcpp::QoS(1).transient_local());
    scene_sub_=node_.create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states",rclcpp::SensorDataQoS(),[this](gazebo_msgs::msg::ModelStates::SharedPtr message) {
      const bool valid=world_->matches(*message,scene_reason_);
      if(scene_seen_ && scene_valid_!=valid){++revision_;invalidate("Gazebo scene revision changed");}
      scene_seen_=true;scene_valid_=valid;scene_time_=actuator_->rosTime();
    });
    plan_service_=node_.create_service<PlanService>("/coupled/plan",[this](PlanService::Request::SharedPtr request,PlanService::Response::SharedPtr response) {
      try {requestPlan(*request,*response);}catch(const std::exception &error){response->accepted=false;response->reason=error.what();invalidate(error.what());}
      RCLCPP_INFO(node_.get_logger(),"Plan task=%s accepted=%d plan=%s reason=%s",request->task_id.c_str(),response->accepted,response->plan_id.c_str(),response->reason.c_str());
    });
    execute_service_=node_.create_service<ExecuteService>("/coupled/execute",[this](ExecuteService::Request::SharedPtr request,ExecuteService::Response::SharedPtr response) {
      try {requestExecution(*request,*response);}catch(const std::exception &error){response->accepted=false;response->reason=error.what();invalidate(error.what());}
    });
    patrol_service_=node_.create_service<PatrolService>("/coupled/start_patrol",[this](PatrolService::Request::SharedPtr request,PatrolService::Response::SharedPtr response) {
      try {requestPatrol(*request,*response);}catch(const std::exception &error){response->accepted=false;response->reason=error.what();invalidate(error.what());}
    });
    stop_service_=node_.create_service<std_srvs::srv::Trigger>("/simulation/stop",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response) {
      invalidate("operator stop");response->success=true;response->message="motion held; prepared execution token invalidated";
    });
    timer_=node_.create_wall_timer(std::chrono::milliseconds(20),[this] {try{tick();}catch(const std::exception &error){invalidate(error.what());}});
  }
  void stop(){if(actuator_)invalidate("runtime shutdown");}
};

CoupledRuntime::CoupledRuntime(const rclcpp::NodeOptions &options):Node("coupled_runtime",options),impl_(std::make_unique<Implementation>(*this,options)) {}
CoupledRuntime::~CoupledRuntime()=default;
std::string CoupledRuntime::modelSignature() const {return impl_->signature();}
void CoupledRuntime::initialize(std::shared_ptr<ActuatorPort> actuator){impl_->initialize(std::move(actuator));}
rclcpp::Node::SharedPtr CoupledRuntime::backendNode() const {return impl_->backend();}
void CoupledRuntime::stop(){impl_->stop();}
}
