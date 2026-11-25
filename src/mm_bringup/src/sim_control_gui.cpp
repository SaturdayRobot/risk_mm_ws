#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/string.hpp>
#include <risk_mm_interfaces/msg/coupled_status.hpp>
#include <risk_mm_interfaces/srv/plan_coupled.hpp>
#include <risk_mm_interfaces/srv/execute_plan.hpp>
#include <risk_mm_interfaces/srv/start_patrol.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tinyxml2.h>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <array>
#include <chrono>
#include <map>
#include <iostream>
#include <QCloseEvent>
#include <cmath>
#include <deque>
#include "risk_mm_runtime/task_goals.hpp"

class ControlPanel final : public QWidget {
  using JointState=sensor_msgs::msg::JointState;
  using Twist=geometry_msgs::msg::TwistStamped;
  using Clock=std::chrono::steady_clock;
  using CoupledStatus=risk_mm_interfaces::msg::CoupledStatus;
  using PlanService=risk_mm_interfaces::srv::PlanCoupled;
  using ExecuteService=risk_mm_interfaces::srv::ExecutePlan;
  using PatrolService=risk_mm_interfaces::srv::StartPatrol;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<JointState>::SharedPtr arm_pub_;
  rclcpp::Publisher<Twist>::SharedPtr base_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::array<QDoubleSpinBox*,6> boxes_{};
  std::array<QLabel*,6> measured_{};
  std::array<double,6> values_{};
  std::array<double,3> velocity_{};
  QLabel *status_,*image_;
  QCheckBox *enabled_;
  QTimer timer_;
  bool initialized_=false,ready_=false,owned_=false,goal_dirty_=false;
  CoupledStatus::SharedPtr coupled_state_;
  QLabel *coupled_label_;
  std::array<QDoubleSpinBox*,3> goal_{};
  std::array<QDoubleSpinBox*,3> speed_{};
  std::array<double,3> planned_speed_{};
  QPushButton *plan_button_,*execute_button_,*patrol_button_;
  QLabel *patrol_label_;
  bool patrol_request_{false};
  std::vector<QPushButton*> manual_buttons_;
  std::map<std::string,QPushButton*> buttons_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gui_status_pub_;
  rclcpp::Client<PlanService>::SharedPtr plan_client_;
  rclcpp::Client<ExecuteService>::SharedPtr execute_client_;
  rclcpp::Client<PatrolService>::SharedPtr patrol_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  Clock::time_point coupled_time_=Clock::time_point::min();
  std::vector<double> initial_goal_;
  enum class PlanStage {Idle,StopService,StopReply,MeasuredStop,PlanService,PlanReply,Backend};
  PlanStage plan_stage_{PlanStage::Idle};
  Clock::time_point stage_time_{},last_status_{};
  double stopped_since_ros_{-1.};
  std::array<double,9> requested_goal_{};
  std::string accepted_plan_;
  unsigned long action_generation_{0};
  bool execute_pending_{false};
  QLabel *request_label_,*drive_feedback_;
  std::vector<risk_mm::TaskGoal> task_goals_;
  QComboBox *preset_selector_;
  QPushButton *preset_apply_;
  QLabel *preset_description_;
  std::string loaded_preset_;
  std::deque<JointState::SharedPtr> joint_queue_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_queue_;
  JointState::SharedPtr stopped_joints_;
  nav_msgs::msg::Odometry::SharedPtr stopped_odom_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> drive_frame_broadcaster_;
  double drive_yaw_in_base_{0.};
  Clock::time_point feedback_=Clock::time_point::min(),status_time_=Clock::time_point::min();
  void send_base() {
    Twist message;message.header.stamp=node_->now();message.header.frame_id="base_link";
    // Intent is expressed in base_control. The existing model/controller API
    // remains in base_link; transform once here instead of changing wheel axes
    // or silently reversing one button independently of the planning goal.
    const double c=std::cos(drive_yaw_in_base_),s=std::sin(drive_yaw_in_base_);
    message.twist.linear.x=c*velocity_[0]-s*velocity_[1];message.twist.linear.y=s*velocity_[0]+c*velocity_[1];
    message.twist.angular.z=velocity_[2];base_pub_->publish(message);
  }
  void stop() {velocity_.fill(0.);enabled_->setChecked(false);if(rclcpp::ok())send_base();}
  std::array<double,9> draft_goal() const {
    std::array<double,9> goal{};
    for(int i=0;i<3;++i)goal[i]=goal_[i]->value();
    goal[2]=std::remainder(goal[2]-drive_yaw_in_base_,2*M_PI);
    for(int i=0;i<6;++i)goal[i+3]=boxes_[i]->value();
    return goal;
  }
  static bool same_goal(const std::array<double,9> &a,const std::array<double,9> &b) {
    for(int i=0;i<9;++i) {
      const double error=i==2?std::remainder(a[i]-b[i],2*M_PI):a[i]-b[i];
      if(std::abs(error)>5.1e-5)return false;
    }
    return true;
  }
  void request_error(const std::string &message) {
    ++action_generation_;
    plan_stage_=PlanStage::Idle;execute_pending_=false;goal_dirty_=true;
    request_label_->setText(QString::fromStdString(message));
    request_label_->setStyleSheet("color: #a00000; font-weight: bold;");
    RCLCPP_WARN(node_->get_logger(),"%s",message.c_str());
  }
  void request_info(const std::string &message) {
    request_label_->setStyleSheet("");request_label_->setText(QString::fromStdString(message));
  }
  void stop_all() {
    stop();goal_dirty_=true;++action_generation_;plan_stage_=PlanStage::Idle;execute_pending_=false;accepted_plan_.clear();
    if(stop_client_->service_is_ready()) {
      request_info("STOP requested; waiting for simulator acknowledgement");
      const auto generation=action_generation_;
      stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
        if(generation!=action_generation_)return;
        try {const auto result=response.get();if(result->success)request_info(result->message);else request_error("STOP rejected: "+result->message);}
        catch(const std::exception &error){request_error(std::string("STOP failed: ")+error.what());}
      });
    }else request_error("STOP service unavailable; simulator acknowledgement missing");
  }
  void plan(bool patrol=false) {
    if(plan_stage_!=PlanStage::Idle)return;
    if(!coupled_state_ || !coupled_state_->map_ready || !coupled_state_->feedback_ready) {
      request_error("Planning unavailable: waiting for a valid scene and measured robot state");return;
    }
    stop();requested_goal_=draft_goal();accepted_plan_.clear();goal_dirty_=true;execute_button_->setEnabled(false);
    patrol_request_=patrol;
    for(int i=0;i<3;++i)planned_speed_[i]=speed_[i]->value();
    ++action_generation_;stage_time_=Clock::now();stopped_since_ros_=-1.;plan_stage_=PlanStage::StopService;
    request_info("Plan clicked: requesting stop before freezing the measured start");
    RCLCPP_INFO(node_->get_logger(),"Plan clicked: goal x=%.4f y=%.4f yaw=%.4f",requested_goal_[0],requested_goal_[1],requested_goal_[2]);
  }
  void planning_response(bool accepted,const std::string &id,const std::string &reason) {
    if(!accepted){request_error("Plan rejected: "+reason);return;}
    accepted_plan_=id;stage_time_=Clock::now();
    if(patrol_request_) {
      plan_stage_=PlanStage::Idle;execute_pending_=true;
      request_info("自主耦合巡检已启动，将在运动中规划后续路段并返回起点。");
    }else {
      plan_stage_=PlanStage::Backend;request_info("Planner accepted "+id+"; computing candidate and full validation");
    }
  }
  bool measured_stopped() const {
    if(!coupled_state_ || !coupled_state_->feedback_ready || !stopped_joints_ || !stopped_odom_)return false;
    const auto now=node_->now().nanoseconds();
    for(const auto &header:{stopped_joints_->header,stopped_odom_->header}) {
      const auto age=now-rclcpp::Time(header.stamp).nanoseconds();if(age<0 || age>500000000)return false;
    }
    std::map<std::string,double> velocity;
    for(size_t i=0;i<std::min(stopped_joints_->name.size(),stopped_joints_->velocity.size());++i)velocity[stopped_joints_->name[i]]=stopped_joints_->velocity[i];
    for(int i=1;i<=6;++i) {
      const auto name="xarm6_joint"+std::to_string(i);
      if(!velocity.count(name) || !std::isfinite(velocity[name]) || std::abs(velocity[name])>.025)return false;
    }
    const auto &v=stopped_odom_->twist.twist;
    return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.angular.z) &&
      std::hypot(v.linear.x,v.linear.y)<.015 && std::abs(v.angular.z)<.025;
  }
  void advance_plan_request() {
    const auto now=Clock::now();const auto generation=action_generation_;
    const auto elapsed=std::chrono::duration<double>(now-stage_time_).count();
    if(plan_stage_!=PlanStage::Idle && plan_stage_!=PlanStage::Backend && elapsed>10.) {
      request_error("Planning request timed out while waiting for service/stop acknowledgement; press Plan to retry");return;
    }
    if(plan_stage_==PlanStage::StopService && stop_client_->service_is_ready()) {
      plan_stage_=PlanStage::StopReply;stage_time_=now;
      stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
        if(generation!=action_generation_)return;
        try {const auto result=response.get();if(!result->success){request_error("Cannot prepare planning: "+result->message);return;}}
        catch(const std::exception &error){request_error(std::string("Stop acknowledgement failed: ")+error.what());return;}
        plan_stage_=PlanStage::MeasuredStop;stage_time_=Clock::now();request_info("Stop acknowledged; waiting for measured base and arm to settle");
      });
    }else if(plan_stage_==PlanStage::MeasuredStop) {
      const double ros_now=node_->now().seconds();
      if(!measured_stopped())stopped_since_ros_=-1.;
      else if(stopped_since_ros_<0.)stopped_since_ros_=ros_now;
      else if(ros_now-stopped_since_ros_>=.2){plan_stage_=PlanStage::PlanService;stage_time_=now;request_info("Measured stop confirmed; connecting to coupled planner");}
    }else if(plan_stage_==PlanStage::PlanService && (patrol_request_?patrol_client_->service_is_ready():plan_client_->service_is_ready())) {
      if(!coupled_state_ || !coupled_state_->map_ready){request_error("Planning scene is invalid: "+(coupled_state_?coupled_state_->reason:std::string("no status")));return;}
      if(patrol_request_) {
        auto request=std::make_shared<PatrolService::Request>();request->task_id="patrol_"+std::to_string(node_->now().nanoseconds());
        request->model_signature=coupled_state_->model_signature;request->map_revision=coupled_state_->map_revision;request->speed_limits=planned_speed_;
        plan_stage_=PlanStage::PlanReply;stage_time_=now;
        patrol_client_->async_send_request(request,[this,generation](rclcpp::Client<PatrolService>::SharedFuture response){
          if(generation!=action_generation_)return;
          try {const auto result=response.get();planning_response(result->accepted,result->mission_id,result->reason);}
          catch(const std::exception &error){request_error(error.what());}
        });
        return;
      }
      auto request=std::make_shared<PlanService::Request>();request->task_id="operator_"+std::to_string(node_->now().nanoseconds());
      request->model_signature=coupled_state_->model_signature;request->map_revision=coupled_state_->map_revision;request->goal=requested_goal_;
      request->speed_limits=planned_speed_;
      plan_stage_=PlanStage::PlanReply;stage_time_=now;request_info("Coupled planning request sent; awaiting acceptance");
      plan_client_->async_send_request(request,[this,generation](rclcpp::Client<PlanService>::SharedFuture response) {
        if(generation!=action_generation_)return;
        try {
          const auto result=response.get();
          RCLCPP_INFO(node_->get_logger(),"Plan response: accepted=%d id=%s reason=%s",result->accepted,result->plan_id.c_str(),result->reason.c_str());
          planning_response(result->accepted,result->plan_id,result->reason);
        }catch(const std::exception &error){request_error(std::string("Planning service failed: ")+error.what());}
      });
    }else if(plan_stage_==PlanStage::Backend) {
      request_info("Planning "+accepted_plan_+" ... "+std::to_string(static_cast<int>(elapsed))+" s (search, optimization, validation)");
    }
  }
  void execute() {
    if(!coupled_state_ || goal_dirty_ || plan_stage_!=PlanStage::Idle || execute_pending_ ||
       coupled_state_->plan_id!=accepted_plan_ || !same_goal(draft_goal(),coupled_state_->goal)) {
      request_error("Execution blocked: validate the current input goal first");return;
    }
    if(!execute_client_->service_is_ready()){request_error("Execution service unavailable");return;}
    auto request=std::make_shared<ExecuteService::Request>();request->plan_id=coupled_state_->plan_id;
    request->model_signature=coupled_state_->model_signature;request->map_revision=coupled_state_->plan_map_revision;
    execute_button_->setEnabled(false);execute_pending_=true;request_info("Execution requested for "+accepted_plan_);
    const auto generation=action_generation_;
    execute_client_->async_send_request(request,[this,generation](rclcpp::Client<ExecuteService>::SharedFuture response) {
      if(generation!=action_generation_)return;
      try {const auto result=response.get();if(result->accepted)request_info(result->reason);else request_error("Execution rejected: "+result->reason);}
      catch(const std::exception &error){request_error(std::string("Execution request failed: ")+error.what());}
    });
  }
  void sync() {enabled_->setChecked(false);for(size_t i=0;i<6;++i)boxes_[i]->setValue(values_[i]);}
  void tick() {
    if(!rclcpp::ok()){close();return;}
    executor_.spin_some(std::chrono::milliseconds(8));
    const auto now=Clock::now();
    const auto ros_stamp=node_->now().nanoseconds();
    while(!joint_queue_.empty() && rclcpp::Time(joint_queue_.front()->header.stamp).nanoseconds()<=ros_stamp) {
      stopped_joints_=joint_queue_.front();joint_queue_.pop_front();
    }
    while(!odom_queue_.empty() && rclcpp::Time(odom_queue_.front()->header.stamp).nanoseconds()<=ros_stamp) {
      stopped_odom_=odom_queue_.front();odom_queue_.pop_front();
    }
    advance_plan_request();
    const bool coupled_fresh=coupled_state_ && now-coupled_time_<std::chrono::seconds(1);
    const bool busy=coupled_fresh && (coupled_state_->phase=="PLANNING" || coupled_state_->phase=="ALIGNING" ||
                                    coupled_state_->phase=="EXECUTING" || coupled_state_->phase=="SETTLING");
    plan_button_->setEnabled(coupled_fresh && coupled_state_->map_ready && coupled_state_->feedback_ready && !busy && plan_stage_==PlanStage::Idle && !execute_pending_);
    patrol_button_->setEnabled(plan_button_->isEnabled());
    execute_button_->setEnabled(coupled_fresh && coupled_state_->ready_to_execute && !goal_dirty_ &&
      plan_stage_==PlanStage::Idle && !execute_pending_ && coupled_state_->plan_id==accepted_plan_ && same_goal(draft_goal(),coupled_state_->goal));
    const bool manual_available=!owned_ && plan_stage_==PlanStage::Idle && !execute_pending_;
    const bool preset_available=coupled_fresh && !busy && plan_stage_==PlanStage::Idle && !execute_pending_;
    preset_selector_->setEnabled(preset_available);
    preset_apply_->setEnabled(preset_available && preset_selector_->currentIndex()>0);
    for(auto *speed:speed_)speed->setEnabled(!busy && plan_stage_==PlanStage::Idle && !execute_pending_);
    enabled_->setEnabled(manual_available);for(auto *button:manual_buttons_)button->setEnabled(manual_available);
    if(owned_)stop();
    if(!initialized_ || !ready_ || now-feedback_>std::chrono::seconds(1) || now-status_time_>std::chrono::seconds(1)) {
      velocity_.fill(0.);enabled_->setChecked(false);status_->setText("HOLD | waiting for fresh simulator feedback");
    }
    if(velocity_[0]!=0. || velocity_[1]!=0. || velocity_[2]!=0.)send_base();
    if(enabled_->isChecked()) {
      JointState message;message.header.stamp=node_->now();
      for(size_t i=0;i<6;++i){message.name.push_back("xarm6_joint"+std::to_string(i+1));message.position.push_back(boxes_[i]->value());message.velocity.push_back(speed_[2]->value());}
      arm_pub_->publish(message);
    }
    if(now-last_status_>std::chrono::milliseconds(500)) {
      last_status_=now;
      if(stopped_odom_) {
        const auto &q=stopped_odom_->pose.pose.orientation;const auto &v=stopped_odom_->twist.twist.linear;
        const double yaw=std::remainder(std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+drive_yaw_in_base_,2*M_PI);
        drive_feedback_->setText(QString("Measured chassis speed toward LiDAR: %1 m/s | world yaw %2 deg")
          .arg(std::cos(yaw)*v.x+std::sin(yaw)*v.y,0,'f',3).arg(yaw*180./M_PI,0,'f',1));
      }
      QJsonObject telemetry;telemetry["stamp_ros_s"]=node_->now().seconds();telemetry["drive_frame"]="base_control";
      telemetry["drive_yaw_in_base_rad"]=drive_yaw_in_base_;telemetry["request"]=request_label_->text();
      telemetry["accepted_plan_id"]=QString::fromStdString(accepted_plan_);telemetry["pending"]=plan_stage_!=PlanStage::Idle;
      telemetry["goal_dirty"]=goal_dirty_;telemetry["backend_phase"]=coupled_state_?QString::fromStdString(coupled_state_->phase):"unknown";
      telemetry["preset_selection"]=preset_selector_->currentData().toString();
      telemetry["loaded_preset"]=QString::fromStdString(loaded_preset_);
      QJsonArray speed_values;for(auto *box:speed_)speed_values.append(box->value());telemetry["speed_limits"]=speed_values;
      QJsonArray target;for(double value:draft_goal())target.append(value);telemetry["backend_goal"]=target;
      QJsonObject buttons;
      for(const auto &[name,button]:buttons_) {
        const auto p=button->mapTo(this,QPoint(0,0));QJsonObject geometry;
        geometry["x"]=p.x();geometry["y"]=p.y();geometry["width"]=button->width();geometry["height"]=button->height();
        geometry["enabled"]=button->isEnabled();buttons[QString::fromStdString(name)]=geometry;
      }
      telemetry["buttons"]=buttons;std_msgs::msg::String message;
      QJsonObject inputs;
      for(int i=0;i<9;++i) {
        QWidget *input=i<3?static_cast<QWidget*>(goal_[i]):static_cast<QWidget*>(boxes_[i-3]);
        const auto p=input->mapTo(this,QPoint(0,0));QJsonObject geometry;
        geometry["x"]=p.x();geometry["y"]=p.y();geometry["width"]=input->width();geometry["height"]=input->height();
        const std::string name=i==0?"world_x":i==1?"world_y":i==2?"front_heading":"joint"+std::to_string(i-2);
        inputs[QString::fromStdString(name)]=geometry;
      }
      const auto p=preset_selector_->mapTo(this,QPoint(0,0));QJsonObject selector;
      selector["x"]=p.x();selector["y"]=p.y();selector["width"]=preset_selector_->width();selector["height"]=preset_selector_->height();
      inputs["preset_selector"]=selector;
      for(int i=0;i<3;++i) {
        const auto p=speed_[i]->mapTo(this,QPoint(0,0));QJsonObject geometry;
        geometry["x"]=p.x();geometry["y"]=p.y();geometry["width"]=speed_[i]->width();geometry["height"]=speed_[i]->height();
        inputs[i==0?"base_speed":i==1?"yaw_speed":"arm_speed"]=geometry;
      }
      telemetry["inputs"]=inputs;
      message.data=QJsonDocument(telemetry).toJson(QJsonDocument::Compact).toStdString();gui_status_pub_->publish(message);
    }
  }
protected:
  bool event(QEvent *event) override {
    if(event->type()==QEvent::WindowDeactivate && enabled_)stop();
    return QWidget::event(event);
  }
  void closeEvent(QCloseEvent *event) override {stop_all();executor_.spin_some();timer_.stop();QWidget::closeEvent(event);}
public:
  explicit ControlPanel(rclcpp::Node::SharedPtr node):node_(std::move(node)),enabled_(nullptr) {
    executor_.add_node(node_);
    const auto drive_config=YAML::LoadFile(ament_index_cpp::get_package_share_directory("mm_bringup")+"/config/manual_control.yaml");
    drive_yaw_in_base_=drive_config["control_frame_yaw_in_base_rad"].as<double>();
    if(!std::isfinite(drive_yaw_in_base_) || std::abs(std::sin(drive_yaw_in_base_))>1e-9)
      throw std::runtime_error("manual control front must be a longitudinal model axis; lateral driving is disabled");
    drive_frame_broadcaster_=std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    geometry_msgs::msg::TransformStamped drive_transform;drive_transform.header.frame_id="base_link";drive_transform.child_frame_id="base_control";
    drive_transform.header.stamp=node_->now();drive_transform.transform.rotation.z=std::sin(drive_yaw_in_base_/2.);
    drive_transform.transform.rotation.w=std::cos(drive_yaw_in_base_/2.);drive_frame_broadcaster_->sendTransform(drive_transform);
    node_->get_parameter_or("initial_goal",initial_goal_,std::vector<double>{});
    if(!initial_goal_.empty()) {
      if(initial_goal_.size()!=9)throw std::runtime_error("initial_goal requires world X/Y/yaw and six joint targets");
      for(double value:initial_goal_)if(!std::isfinite(value))throw std::runtime_error("nonfinite initial_goal");
    }
    setWindowTitle("Rangermini + xArm6 | C++ Gazebo control");
    arm_pub_=node_->create_publisher<JointState>("/simulation/arm_command",10);
    base_pub_=node_->create_publisher<Twist>("/simulation/cmd_vel",10);
    gui_status_pub_=node_->create_publisher<std_msgs::msg::String>("/simulation/gui_status",1);
    plan_client_=node_->create_client<PlanService>("/coupled/plan");
    execute_client_=node_->create_client<ExecuteService>("/coupled/execute");
    patrol_client_=node_->create_client<PatrolService>("/coupled/start_patrol");
    stop_client_=node_->create_client<std_srvs::srv::Trigger>("/simulation/stop");
    auto *root=new QVBoxLayout(this);status_=new QLabel("Waiting for measured Gazebo state...");root->addWidget(status_);
    root->addWidget(new QLabel("Hold to drive; release to stop. Forward = LiDAR side (+base_control X)."));
    auto *speeds=new QGridLayout;
    const auto hard=YAML::LoadFile(ament_index_cpp::get_package_share_directory("mm_bringup")+"/config/coupled_execution.yaml")["/**"]["ros__parameters"];
    const std::array<const char*,3> keys{"default_base_speed_m_s","default_base_yaw_rad_s","default_arm_speed_rad_s"};
    const std::array<const char*,3> caps{"base_speed_max_m_s","base_yaw_max_rad_s","arm_rate_rad_s"};
    const std::array<const char*,3> validation_keys{"validation.base_velocity","validation.base_yaw_rate","validation.arm_velocity"};
    std::vector<double> initial_speeds;
    node_->get_parameter_or("initial_speed_limits",initial_speeds,std::vector<double>{});
    if(!initial_speeds.empty() && initial_speeds.size()!=3)throw std::runtime_error("three initial speed limits required");
    for(int i=0;i<3;++i) {
      speed_[i]=new QDoubleSpinBox;speed_[i]->setDecimals(2);
      speed_[i]->setRange(.02,std::min(drive_config[caps[i]].as<double>(),hard[validation_keys[i]].as<double>()));
      speed_[i]->setSingleStep(.05);speed_[i]->setValue(drive_config[keys[i]].as<double>());
      if(!initial_speeds.empty()) {
        if(!std::isfinite(initial_speeds[i]) || initial_speeds[i]<speed_[i]->minimum() || initial_speeds[i]>speed_[i]->maximum())
          throw std::runtime_error("saved GUI speed exceeds current configured limits");
        speed_[i]->setValue(initial_speeds[i]);
      }
      speeds->addWidget(new QLabel(i==0?"线速度上限 (m/s)":i==1?"角速度上限 (rad/s)":"关节速度上限 (rad/s)"),0,i);
      speeds->addWidget(speed_[i],1,i);
      connect(speed_[i],qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){
        stop();goal_dirty_=true;accepted_plan_.clear();execute_button_->setEnabled(false);
        request_info("速度设置已更改，请重新 Plan；局部曲率和加速度约束仍会限制实际速度。");
      });
    }
    root->addLayout(speeds);
    auto *pad=new QGridLayout;
    auto button=[&](const char *label,int row,int col,std::array<double,3> command) {
      auto *b=new QPushButton(label);pad->addWidget(b,row,col);manual_buttons_.push_back(b);
      buttons_[command[0]>0.?"forward":command[0]<0.?"back":command[2]>0.?"left":"right"]=b;
      connect(b,&QPushButton::pressed,this,[this,command]{velocity_={command[0]*speed_[0]->value(),0.,command[2]*speed_[1]->value()};});
      connect(b,&QPushButton::released,this,[this]{velocity_.fill(0.);send_base();});
    };
    button("Forward (LiDAR)",0,1,{1.,0.,0.});button("Back",2,1,{-1.,0.,0.});
    button("Rotate left",0,0,{0.,0.,1.});button("Rotate right",0,2,{0.,0.,-1.});
    auto *stop_button=new QPushButton("STOP / HOLD");pad->addWidget(stop_button,1,1);connect(stop_button,&QPushButton::clicked,this,[this]{stop_all();});root->addLayout(pad);
    buttons_["stop"]=stop_button;
    drive_feedback_=new QLabel("Waiting for measured chassis speed");drive_feedback_->setWordWrap(true);root->addWidget(drive_feedback_);
    auto *grid=new QGridLayout;grid->addWidget(new QLabel("Joint"),0,0);grid->addWidget(new QLabel("Target (rad)"),0,1);grid->addWidget(new QLabel("Measured (rad)"),0,2);
    tinyxml2::XMLDocument model;
    const auto file=ament_index_cpp::get_package_share_directory("rangermini_description")+"/urdf/rangermini_xarm6.urdf";
    if(model.LoadFile(file.c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("Missing robot URDF");
    for(size_t i=0;i<6;++i) {
      const std::string name="xarm6_joint"+std::to_string(i+1);boxes_[i]=new QDoubleSpinBox;boxes_[i]->setDecimals(4);boxes_[i]->setSingleStep(.02);
      for(auto *j=model.RootElement()->FirstChildElement("joint");j;j=j->NextSiblingElement("joint"))if(name==j->Attribute("name")) {
        auto *limit=j->FirstChildElement("limit");boxes_[i]->setRange(limit->DoubleAttribute("lower"),limit->DoubleAttribute("upper"));
      }
      measured_[i]=new QLabel("--");grid->addWidget(new QLabel(QString::fromStdString(name)),i+1,0);grid->addWidget(boxes_[i],i+1,1);grid->addWidget(measured_[i],i+1,2);
      connect(boxes_[i],qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){goal_dirty_=true;});
    }root->addLayout(grid);
    enabled_=new QCheckBox("Send arm targets（按上方关节速度上限）");root->addWidget(enabled_);
    auto *copy=new QPushButton("Copy measured joints to targets");connect(copy,&QPushButton::clicked,this,[this]{if(initialized_)sync();});root->addWidget(copy);
    auto *goals=new QGridLayout;
    for(int i=0;i<3;++i) {
      goal_[i]=new QDoubleSpinBox;goal_[i]->setDecimals(4);goal_[i]->setRange(i==2?-M_PI:-11.,i==2?M_PI:11.);
      goal_[i]->setSingleStep(i==2?.1:.05);goals->addWidget(new QLabel(i==0?"World X (m)":i==1?"World Y (m)":"Front heading (rad)"),0,i);goals->addWidget(goal_[i],1,i);
      connect(goal_[i],qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){goal_dirty_=true;});
    }
    auto *ahead=new QPushButton("Goal: 0.6 m ahead from measured pose");
    buttons_["ahead"]=ahead;
    connect(ahead,&QPushButton::clicked,this,[this]{if(!coupled_state_)return;const auto &s=coupled_state_->state;
      const double heading=std::remainder(s[2]+drive_yaw_in_base_,2*M_PI);
      goal_[0]->setValue(s[0]+.6*std::cos(heading));goal_[1]->setValue(s[1]+.6*std::sin(heading));goal_[2]->setValue(heading);sync();});
    root->addLayout(goals);root->addWidget(ahead);
    const auto bringup=ament_index_cpp::get_package_share_directory("mm_bringup");
    std::string world_name;node_->get_parameter_or("world_name",world_name,std::string("s6_factory_inspection_v1"));
    std::string metadata_file,goals_file;
    node_->get_parameter_or("world_metadata_file",metadata_file,bringup+"/worlds/"+world_name+".json");
    node_->get_parameter_or("task_goals_file",goals_file,bringup+"/config/goal_presets.yaml");
    const auto metadata=YAML::LoadFile(metadata_file);
    task_goals_=risk_mm::loadTaskGoals(goals_file,world_name,metadata["world_sha256"].as<std::string>());
    for(const auto &preset:task_goals_)for(int i=0;i<9;++i) {
      const auto *box=i<3?goal_[i]:boxes_[i-3];
      if(preset.input[i]<box->minimum() || preset.input[i]>box->maximum())
        throw std::runtime_error("task preset outside URDF/input limits: "+preset.id);
    }
    auto *presets=new QHBoxLayout;preset_selector_=new QComboBox;
    preset_selector_->addItem("选择工厂任务目标（不会自动运动）",QString());
    for(const auto &preset:task_goals_)preset_selector_->addItem(QString::fromStdString(preset.label),QString::fromStdString(preset.id));
    preset_apply_=new QPushButton("载入目标");buttons_["load_preset"]=preset_apply_;
    presets->addWidget(preset_selector_,1);presets->addWidget(preset_apply_);root->addLayout(presets);
    preset_description_=new QLabel("预设为固定世界目标；从当前实测状态重新规划，校验通过后才可执行。");
    preset_description_->setWordWrap(true);root->addWidget(preset_description_);
    connect(preset_selector_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this](int index){
      if(index>0)preset_description_->setText(QString::fromStdString(task_goals_.at(index-1).description));
    });
    connect(preset_apply_,&QPushButton::clicked,this,[this]{
      const int index=preset_selector_->currentIndex();if(index<=0)return;
      const auto &preset=task_goals_.at(index-1);enabled_->setChecked(false);
      for(int i=0;i<3;++i)goal_[i]->setValue(preset.input[i]);
      for(int i=0;i<6;++i)boxes_[i]->setValue(preset.input[i+3]);
      loaded_preset_=preset.id;accepted_plan_.clear();goal_dirty_=true;execute_button_->setEnabled(false);
      request_info("已载入 "+preset.label+"；请点击 Plan，从当前状态规划。");
    });
    auto *actions=new QGridLayout;plan_button_=new QPushButton("1. Plan coupled");execute_button_=new QPushButton("2. Execute validated plan");
    buttons_["plan"]=plan_button_;buttons_["execute"]=execute_button_;
    actions->addWidget(plan_button_,0,0);actions->addWidget(execute_button_,0,1);root->addLayout(actions);
    connect(plan_button_,&QPushButton::clicked,this,[this]{plan();});connect(execute_button_,&QPushButton::clicked,this,[this]{execute();});
    coupled_label_=new QLabel("Waiting for coupled scene and measured state...");coupled_label_->setWordWrap(true);root->addWidget(coupled_label_);
    request_label_=new QLabel("Set a goal, then Plan. The current goal must be validated before Execute.");request_label_->setWordWrap(true);root->addWidget(request_label_);
    patrol_button_=new QPushButton("自主巡检：从当前起点 → 各巡检点 → 返回起点");buttons_["patrol"]=patrol_button_;
    connect(patrol_button_,&QPushButton::clicked,this,[this]{plan(true);});root->addWidget(patrol_button_);
    patrol_label_=new QLabel;patrol_label_->setWordWrap(true);root->addWidget(patrol_label_);
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>("/coupled/patrol_status",10,[this](std_msgs::msg::String::SharedPtr message){
      const auto state=QJsonDocument::fromJson(QByteArray::fromStdString(message->data)).object();
      patrol_label_->setText(QString("巡检 %1 | 已通过 %2/%3 | 上次规划 %4 ms | 规划停车 %5 次")
        .arg(state["phase"].toString()).arg(state["visited"].toInt()).arg(state["total"].toInt())
        .arg(state["planning_ms"].toDouble(),0,'f',1).arg(state["planning_stops"].toInt()));
    }));
    image_=new QLabel("Waiting for head camera image...");image_->setAlignment(Qt::AlignCenter);image_->setMinimumWidth(400);image_->setFixedHeight(170);root->addWidget(image_);
    root->addWidget(new QLabel("Sampled full-mesh validation; no lateral base command."));
    subscriptions_.push_back(node_->create_subscription<CoupledStatus>("/coupled/status",10,[this](CoupledStatus::SharedPtr m){
      const bool first=!coupled_state_;coupled_state_=m;coupled_time_=Clock::now();
      if(first && initial_goal_.empty()) {
        goal_[0]->setValue(m->state[0]);goal_[1]->setValue(m->state[1]);goal_[2]->setValue(std::remainder(m->state[2]+drive_yaw_in_base_,2*M_PI));
      }
      coupled_label_->setText(QString("%1 | candidate %2 | validated %3 | started %4 | completed %5\n%6")
        .arg(QString::fromStdString(m->plan_id+" "+m->phase)).arg(m->candidate_generated).arg(m->validation_passed)
        .arg(m->execution_started).arg(m->execution_completed).arg(QString::fromStdString(m->reason)));
      if(m->plan_id==accepted_plan_) {
        if(plan_stage_==PlanStage::Backend && m->phase=="READY" && m->ready_to_execute) {
          plan_stage_=PlanStage::Idle;goal_dirty_=!same_goal(draft_goal(),requested_goal_);
          request_info(goal_dirty_?"Plan validated for the previous input; goal was edited, please replan":"Validated "+accepted_plan_+"; Execute is available for this input goal");
        }else if(m->phase=="REJECTED" || m->phase=="HOLD") {
          if(plan_stage_==PlanStage::Backend || execute_pending_)request_error("Planner/execution held: "+m->reason);
        }else if(m->phase=="COMPLETED") {execute_pending_=false;request_info("Measured goal reached and stopped: "+accepted_plan_);}
      }
    }));
    subscriptions_.push_back(node_->create_subscription<JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](JointState::SharedPtr m){
      joint_queue_.push_back(m);while(joint_queue_.size()>512)joint_queue_.pop_front();
      std::map<std::string,double> values;for(size_t i=0;i<std::min(m->name.size(),m->position.size());++i)values[m->name[i]]=m->position[i];
      for(size_t i=0;i<6;++i)if(!values.count("xarm6_joint"+std::to_string(i+1)))return;
      for(size_t i=0;i<6;++i){values_[i]=values.at("xarm6_joint"+std::to_string(i+1));measured_[i]->setText(QString::number(values_[i],'f',4));}
      feedback_=Clock::now();if(!initialized_){
        sync();initialized_=true;
        if(!initial_goal_.empty()) {
          for(int i=0;i<3;++i)goal_[i]->setValue(initial_goal_[i]);
          goal_[2]->setValue(std::remainder(initial_goal_[2]+drive_yaw_in_base_,2*M_PI));
          for(int i=0;i<6;++i)boxes_[i]->setValue(initial_goal_[i+3]);
        }
      }
    }));
    subscriptions_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>("/ground_truth/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr m){
      if(m->header.frame_id!="world" || m->child_frame_id!="base_link")return;
      odom_queue_.push_back(m);while(odom_queue_.size()>128)odom_queue_.pop_front();
    }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>("/simulation/control_status",10,[this](std_msgs::msg::String::SharedPtr m){
      auto status=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();ready_=status["ready"].toBool();status_time_=Clock::now();
      owned_=!status["owner_plan"].toString().isEmpty();
      status_->setText((ready_?"READY | ":"HOLD | ")+status["reason"].toString());
    }));
    subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::Image>("/camera/head/image_raw",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Image::SharedPtr m){
      if((m->encoding!="rgb8" && m->encoding!="bgr8") || m->data.size()<m->height*m->step)return;
      QImage image(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888);auto copy=image.copy();if(m->encoding=="bgr8")copy=copy.rgbSwapped();
      image_->setPixmap(QPixmap::fromImage(copy).scaled(image_->size(),Qt::KeepAspectRatio,Qt::SmoothTransformation));
    }));
    resize(620,860);connect(&timer_,&QTimer::timeout,this,[this]{tick();});timer_.start(50);
  }
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);QApplication app(argc,argv);
  try {auto node=std::make_shared<rclcpp::Node>("sim_control_gui",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));ControlPanel panel(node);panel.show();app.exec();}
  catch(const std::exception &error){std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();
}
