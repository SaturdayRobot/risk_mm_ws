#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <mm_model/wheel_kinematics.hpp>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <iostream>
#include <deque>
#include <sstream>
#include <mutex>
#include "mm_bringup/sim_control.hpp"

namespace risk_mm {
class SimControl final : public rclcpp::Node, public ActuatorPort {
  using Joints=sensor_msgs::msg::JointState;
  using Twist=geometry_msgs::msg::TwistStamped;
  using Array=std_msgs::msg::Float64MultiArray;
  std::vector<std::string> arm_,steer_,wheels_;
  std::vector<std::pair<double,double>> bounds_;
  std::array<double,8> positions_{};
  std::array<double,4> steer_applied_{},wheel_applied_{};
  std::vector<double> arm_applied_;
  std::vector<double> arm_hold_;
  Joints::SharedPtr feedback_,target_;
  std::deque<Joints::SharedPtr> pending_feedback_;
  std::deque<Joints::SharedPtr> pending_arm_;
  std::deque<Twist::SharedPtr> pending_base_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> pending_odom_;
  Twist::SharedPtr base_;
  YAML::Node config_;
  double last_tick_=-1.,last_status_=-1.,wheel_radius_=0.;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<Array>::SharedPtr arm_pub_,steer_pub_,wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Joints>::SharedPtr feedback_sub_,arm_sub_;
  rclcpp::Subscription<Twist>::SharedPtr base_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex mutex_;
  std::string model_signature_,owner_plan_,owner_fault_;
  unsigned long owner_map_revision_{0};
  nav_msgs::msg::Odometry::SharedPtr odom_;
  std::array<double,3> base_velocity_{};
  bool base_velocity_valid_{false},coupled_received_{false};
  double coupled_command_ros_s_{0};
  ActuatorCommand coupled_command_;
  double auto_arm_rate_{.35},auto_steer_rate_{M_PI_2},auto_wheel_rate_{10.},auto_wheel_accel_{5.};
  double parameter(const char *key) const {return config_[key].as<double>();}
  bool fresh(const std_msgs::msg::Header &header,double age) {
    const auto delta=now().nanoseconds()-rclcpp::Time(header.stamp).nanoseconds();
    return delta>=0 && delta<=static_cast<int64_t>(age*1e9);
  }
  static double approach(double current,double goal,double step) {return current+std::clamp(goal-current,-step,step);}
  static Array array(const std::vector<double> &values) {Array message;message.data=values;return message;}
  template<size_t N> static Array array(const std::array<double,N> &values) {
    return array(std::vector<double>(values.begin(),values.end()));
  }
  template<typename Message>
  bool defer_until_clock(const std::shared_ptr<Message> &message,
                         std::deque<std::shared_ptr<Message>> &pending) {
    const auto delta=now().nanoseconds()-rclcpp::Time(message->header.stamp).nanoseconds();
    if(delta>=0)return false;
    // DDS topics may arrive before the corresponding 100 Hz /clock update.
    // Such data waits in a bounded queue; it is never accepted as current.
    if(delta>=-static_cast<int64_t>(parameter("feedback_timeout_s")*1e9)) {
      pending.push_back(message);
      while(pending.size()>512)pending.pop_front();
    }
    return true;
  }
  template<typename Message>
  std::shared_ptr<Message> available(std::deque<std::shared_ptr<Message>> &pending) {
    std::shared_ptr<Message> result;
    const auto stamp=now().nanoseconds();
    while(!pending.empty() && rclcpp::Time(pending.front()->header.stamp).nanoseconds()<=stamp) {
      result=pending.front();pending.pop_front();
    }
    return result;
  }
  void receive_state(const Joints::SharedPtr message) {
    std::map<std::string,size_t> index;
    for(size_t i=0;i<message->name.size();++i)index.emplace(message->name[i],i);
    bool valid=index.size()==message->name.size();
    for(const auto &group:{arm_,steer_,wheels_})for(const auto &name:group) {
      if(!index.count(name)){valid=false;continue;}
      auto i=index[name];valid=valid && i<message->position.size() && i<message->velocity.size();
      if(i<message->position.size() && i<message->velocity.size())
        valid=valid && std::isfinite(message->position[i]) && std::isfinite(message->velocity[i]);
    }
    if(!valid){feedback_.reset();pending_feedback_.clear();return;}
    if(defer_until_clock(message,pending_feedback_))return;
    if(!fresh(message->header,parameter("feedback_timeout_s"))){feedback_.reset();return;}
    feedback_=message;
    if(arm_applied_.empty()) {
      for(auto &name:arm_)arm_applied_.push_back(message->position[index[name]]);
      for(size_t i=0;i<4;++i)steer_applied_[i]=message->position[index[steer_[i]]];
    }
  }
  void receive_arm(const Joints::SharedPtr message) {
    if(!owner_plan_.empty())return;
    auto reject=[this]{target_.reset();pending_arm_.clear();};
    if(message->name.size()!=6 || message->position.size()!=6){reject();return;}
    std::map<std::string,double> values;
    for(size_t i=0;i<6;++i)values.emplace(message->name[i],message->position[i]);
    if(values.size()!=6){reject();return;}
    auto ordered=std::make_shared<Joints>();ordered->header=message->header;ordered->name=arm_;
    for(size_t i=0;i<6;++i) {
      if(!values.count(arm_[i])){reject();return;}
      double v=values.at(arm_[i]);if(!std::isfinite(v) || v<bounds_[i].first || v>bounds_[i].second){reject();return;}
      ordered->position.push_back(v);
      const auto index=static_cast<size_t>(std::find(message->name.begin(),message->name.end(),arm_[i])-message->name.begin());
      const double rate=message->velocity.empty()?parameter("arm_rate_rad_s"):
          (message->velocity.size()==6?message->velocity[index]:-1.);
      if(!std::isfinite(rate) || rate<=0. || rate>parameter("arm_rate_rad_s")){reject();return;}
      ordered->velocity.push_back(rate);
    }
    if(defer_until_clock(ordered,pending_arm_))return;
    if(!fresh(ordered->header,parameter("command_timeout_s"))){reject();return;}
    target_=ordered;
  }
  void receive_base(const Twist::SharedPtr message) {
    if(!owner_plan_.empty())return;
    auto &t=message->twist;
    auto reject=[this]{base_.reset();pending_base_.clear();};
    if(message->header.frame_id!="base_link"){reject();return;}
    for(double value:{t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z})if(!std::isfinite(value)){reject();return;}
    if(std::abs(t.linear.y)+std::abs(t.linear.z)+std::abs(t.angular.x)+std::abs(t.angular.y)>1e-9 ||
       std::hypot(t.linear.x,t.linear.y)>parameter("base_speed_max_m_s") ||
       std::abs(t.angular.z)>parameter("base_yaw_max_rad_s")){reject();return;}
    if(defer_until_clock(message,pending_base_))return;
    if(!fresh(message->header,parameter("command_timeout_s"))){reject();return;}
    base_=message;
  }
  void receive_odom(const nav_msgs::msg::Odometry::SharedPtr message) {
    auto &p=message->pose.pose.position;auto &q=message->pose.pose.orientation;
    if(message->header.frame_id!="world" || message->child_frame_id!="base_link" ||
       std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>1e-3) {
      odom_.reset();base_velocity_valid_=false;pending_odom_.clear();return;
    }
    for(double value:{p.x,p.y,p.z,q.x,q.y,q.z,q.w})if(!std::isfinite(value))return;
    if(defer_until_clock(message,pending_odom_))return;
    if(!fresh(message->header,parameter("feedback_timeout_s")))return;
    if(odom_) {
      const double dt=(rclcpp::Time(message->header.stamp)-rclcpp::Time(odom_->header.stamp)).seconds();
      if(dt<=0.)return;
      auto yaw=[](const auto &q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
      base_velocity_[0]=(p.x-odom_->pose.pose.position.x)/dt;
      base_velocity_[1]=(p.y-odom_->pose.pose.position.y)/dt;
      base_velocity_[2]=std::remainder(yaw(q)-yaw(odom_->pose.pose.orientation),2*M_PI)/dt;
      base_velocity_valid_=true;
    }
    odom_=message;
    geometry_msgs::msg::TransformStamped t;t.header=message->header;t.header.frame_id="world";t.child_frame_id="base_link";
    t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z;t.transform.rotation=q;tf_->sendTransform(t);
  }
  void tick() {
    const double stamp=now().seconds();double dt=last_tick_<0?.02:std::clamp(stamp-last_tick_,0.,.05);
    if(last_tick_>=0 && stamp<last_tick_){feedback_.reset();odom_.reset();base_velocity_valid_=false;pending_feedback_.clear();pending_odom_.clear();pending_arm_.clear();pending_base_.clear();base_.reset();target_.reset();last_status_=-1.;if(!owner_plan_.empty())owner_fault_="ROS clock moved backwards";}
    last_tick_=stamp;
    if(auto message=available(pending_feedback_))receive_state(message);
    if(auto message=available(pending_odom_))receive_odom(message);
    if(auto message=available(pending_arm_))receive_arm(message);
    if(auto message=available(pending_base_))receive_base(message);
    const bool ready=feedback_ && fresh(feedback_->header,parameter("feedback_timeout_s"));
    const bool arm_active=ready && target_ && fresh(target_->header,parameter("command_timeout_s"));
    const bool base_active=ready && base_ && fresh(base_->header,parameter("command_timeout_s"));
    if(ready && !owner_plan_.empty()) {
      if(coupled_received_ && stamp-coupled_command_ros_s_>.15)owner_fault_="coupled command timeout";
      if(coupled_received_ && owner_fault_.empty()) {
        std::map<std::string,double> measured;
        for(size_t i=0;i<feedback_->name.size();++i)measured[feedback_->name[i]]=feedback_->position[i];
        bool aligned=true;
        for(size_t i=0;i<6;++i) {
          arm_applied_[i]=approach(arm_applied_[i],coupled_command_.arm[i],auto_arm_rate_*dt);
          arm_hold_[i]=arm_applied_[i];
        }
        for(size_t i=0;i<4;++i) {
          steer_applied_[i]=approach(steer_applied_[i],coupled_command_.steer[i],auto_steer_rate_*dt);
          aligned=aligned && std::abs(measured[steer_[i]]-coupled_command_.steer[i])<.04;
        }
        for(size_t i=0;i<4;++i)wheel_applied_[i]=approach(wheel_applied_[i],aligned?coupled_command_.wheel_velocity[i]:0.,auto_wheel_accel_*dt);
      } else wheel_applied_.fill(0.);
    } else if(ready) {
      std::map<std::string,double> measured;for(size_t i=0;i<feedback_->name.size();++i)measured[feedback_->name[i]]=feedback_->position[i];
      for(size_t i=0;i<6;++i) {
        const double rate=arm_active?target_->velocity[i]:parameter("arm_rate_rad_s");
        arm_applied_[i]=approach(arm_applied_[i],arm_active?target_->position[i]:arm_hold_[i],rate*dt);
        if(arm_active)arm_hold_[i]=arm_applied_[i];
      }
      auto steer_goal=steer_applied_;std::array<double,4> wheel_goal{};
      if(base_active) {
        const auto &t=base_->twist;
        const auto solution=mm_model::inverse(t.linear.x,0.,t.angular.z,positions_,wheel_radius_,
                                              parameter("wheel_rate_max_rad_s"),steer_applied_);
        steer_goal=solution.steer;wheel_goal=solution.speed;
      }
      bool aligned=true;
      for(size_t i=0;i<4;++i) {
        steer_applied_[i]=approach(steer_applied_[i],steer_goal[i],parameter("steer_rate_rad_s")*dt);
        aligned=aligned && std::abs(measured[steer_[i]]-steer_goal[i])<.04;
      }
      for(size_t i=0;i<4;++i)wheel_applied_[i]=base_active?
        approach(wheel_applied_[i],aligned?wheel_goal[i]:0.,parameter("wheel_accel_rad_s2")*dt):0.;
    } else wheel_applied_.fill(0.);
    if(!arm_applied_.empty())arm_pub_->publish(array(arm_applied_));
    steer_pub_->publish(array(steer_applied_));wheel_pub_->publish(array(wheel_applied_));
    if(stamp-last_status_>=.1) {
      std::ostringstream out;out<<std::boolalpha<<"{\"ready\":"<<ready<<",\"arm_command_active\":"<<arm_active
        <<",\"base_command_active\":"<<base_active<<",\"owner_plan\":\""<<owner_plan_<<"\",\"reason\":\""<<(ready?(owner_plan_.empty()?"manual Gazebo control":"coupled actuator ownership"):"missing/stale joint feedback")<<"\",\"wheel_applied\":[";
      for(size_t i=0;i<4;++i) {
        out<<(i?",":"")<<wheel_applied_[i];
      }
      out<<"]}";
      std_msgs::msg::String status;status.data=out.str();status_pub_->publish(status);last_status_=stamp;
    }
  }
  RobotFeedback snapshot() const {
    RobotFeedback result;result.model_signature=model_signature_;result.owner_plan=owner_plan_;result.owner_fault=owner_fault_;
    auto &self=const_cast<SimControl&>(*this);
    if(!feedback_ || !odom_ || !base_velocity_valid_ || !self.fresh(feedback_->header,parameter("feedback_timeout_s")) ||
       !self.fresh(odom_->header,parameter("feedback_timeout_s"))) {result.reason="missing/stale measured joints or odometry";return result;}
    std::map<std::string,size_t> index;
    for(size_t i=0;i<feedback_->name.size();++i)index[feedback_->name[i]]=i;
    const auto &p=odom_->pose.pose.position;const auto &q=odom_->pose.pose.orientation;
    result.position[0]=p.x;result.position[1]=p.y;result.base_z=p.z;
    result.position[2]=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    result.roll=std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));
    result.pitch=std::asin(std::clamp(2*(q.w*q.y-q.z*q.x),-1.,1.));
    for(size_t i=0;i<3;++i)result.velocity[i]=base_velocity_[i];
    for(size_t i=0;i<6;++i){result.position[i+3]=feedback_->position[index.at(arm_[i])];result.velocity[i+3]=feedback_->velocity[index.at(arm_[i])];}
    for(size_t i=0;i<4;++i){result.steer[i]=feedback_->position[index.at(steer_[i])];result.wheel_position[i]=feedback_->position[index.at(wheels_[i])];result.wheel_velocity[i]=feedback_->velocity[index.at(wheels_[i])];}
    result.stamp_ros_s=std::min(rclcpp::Time(feedback_->header.stamp).seconds(),rclcpp::Time(odom_->header.stamp).seconds());
    result.valid=true;return result;
  }
public:
  double rosTime() const override {return now().seconds();}
  RobotFeedback feedback() const override {std::lock_guard<std::mutex> lock(mutex_);return snapshot();}
  bool reserve(const std::string &id,const std::string &signature,unsigned long revision,std::string &reason) override {
    std::lock_guard<std::mutex> lock(mutex_);const auto state=snapshot();
    if(id.empty() || signature.empty() || signature!=model_signature_ || revision==0 || !state.valid) {reason="invalid owner/model or stale measured state";return false;}
    if(!owner_plan_.empty() && owner_plan_!=id){reason="actuators owned by another plan";return false;}
    if(std::hypot(state.velocity[0],state.velocity[1])>.02 || std::abs(state.velocity[2])>.03) {reason="measured base stop required";return false;}
    for(int i=3;i<9;++i)if(std::abs(state.velocity[i])>.03){reason="measured arm stop required";return false;}
    owner_plan_=id;owner_map_revision_=revision;owner_fault_.clear();coupled_received_=false;
    base_.reset();target_.reset();pending_base_.clear();pending_arm_.clear();wheel_applied_.fill(0.);
    // Keep the published position target continuous across ownership changes.
    // Gravity deflection in measured joints is not a new position command.
    arm_hold_=arm_applied_;
    reason.clear();return true;
  }
  bool apply(const ActuatorCommand &command,std::string &reason) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(command.plan_id!=owner_plan_ || owner_plan_.empty() || command.model_signature!=model_signature_ ||
       command.map_revision!=owner_map_revision_) {reason="coupled actuator owner/model/map mismatch";return false;}
    if(!owner_fault_.empty()) {reason="coupled actuator fault: "+owner_fault_+"; command age_s="+std::to_string(rosTime()-coupled_command_ros_s_);return false;}
    const auto state=snapshot();
    if(!state.valid) {reason="coupled command feedback invalid: "+state.reason;return false;}
    for(size_t i=0;i<6;++i)if(!std::isfinite(command.arm[i]) || command.arm[i]<bounds_[i].first || command.arm[i]>bounds_[i].second){reason="invalid coupled arm command";return false;}
    for(size_t i=0;i<4;++i)if(!std::isfinite(command.steer[i]) || std::abs(command.steer[i])>M_PI_2+1e-6 || !std::isfinite(command.wheel_velocity[i]) || std::abs(command.wheel_velocity[i])>auto_wheel_rate_+1e-6){reason="invalid coupled wheel command";return false;}
    coupled_command_=command;coupled_command_ros_s_=rosTime();coupled_received_=true;reason.clear();return true;
  }
  void hold(const std::string &) override {
    std::lock_guard<std::mutex> lock(mutex_);owner_plan_.clear();owner_fault_.clear();coupled_received_=false;
    wheel_applied_.fill(0.);base_.reset();target_.reset();pending_base_.clear();pending_arm_.clear();
    if(!arm_applied_.empty())arm_hold_=arm_applied_;
    if(rclcpp::ok())wheel_pub_->publish(array(wheel_applied_));
  }
  SimControl(const rclcpp::NodeOptions &options,const std::string &signature):Node("sim_control",options),model_signature_(signature) {
    auto share=[](const std::string &p){return ament_index_cpp::get_package_share_directory(p);};
    config_=YAML::LoadFile(share("mm_bringup")+"/config/manual_control.yaml");
    arm_hold_=YAML::LoadFile(share("mm_bringup")+"/config/robot.yaml")["display_arm"].as<std::vector<double>>();
    if(arm_hold_.size()!=6)throw std::runtime_error("Initial arm hold requires six joints");
    get_parameter_or("validation.arm_velocity",auto_arm_rate_,.35);
    get_parameter_or("validation.steer_rate",auto_steer_rate_,M_PI_2);
    get_parameter_or("validation.wheel_velocity",auto_wheel_rate_,10.);
    get_parameter_or("validation.wheel_acceleration",auto_wheel_accel_,5.);
    const auto collision=YAML::LoadFile(share("mm_bringup")+"/config/simulation_collision.yaml");wheel_radius_=collision["tire"]["radius"].as<double>();
    tinyxml2::XMLDocument doc;
    if(doc.LoadFile((share("rangermini_description")+"/urdf/rangermini_xarm6.urdf").c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("Missing robot URDF");
    for(int i=1;i<=6;++i)arm_.push_back("xarm6_joint"+std::to_string(i));
    for(auto *w:{"fl","fr","rl","rr"}){steer_.push_back(std::string("joint_steer_")+w);wheels_.push_back(std::string("joint_wheel_")+w);}
    for(const auto &name:arm_)for(auto *j=doc.RootElement()->FirstChildElement("joint");j;j=j->NextSiblingElement("joint"))if(name==j->Attribute("name")) {
      auto *l=j->FirstChildElement("limit");bounds_.emplace_back(l->DoubleAttribute("lower"),l->DoubleAttribute("upper"));
    }
    for(size_t i=0;i<4;++i) {
      std::array<double,3> steer_position{},wheel_position{},steer_rpy{};
      for(auto *j=doc.RootElement()->FirstChildElement("joint");j;j=j->NextSiblingElement("joint")) {
        const std::string name=j->Attribute("name");
        if(name!=steer_[i] && name!=wheels_[i])continue;
        auto *origin=j->FirstChildElement("origin");
        auto &p=name==steer_[i]?steer_position:wheel_position;
        std::istringstream(origin->Attribute("xyz"))>>p[0]>>p[1]>>p[2];
        if(name==steer_[i])std::istringstream(origin->Attribute("rpy"))>>steer_rpy[0]>>steer_rpy[1]>>steer_rpy[2];
      }
      // Contact center, including the local tire offset and steering-frame yaw.
      const auto suffix=wheels_[i].substr(std::string("joint_wheel_").size());
      wheel_position[1]+=collision["tire"]["center_y"][suffix].as<double>();
      const double c=std::cos(steer_rpy[2]),s=std::sin(steer_rpy[2]);
      positions_[2*i]=steer_position[0]+c*wheel_position[0]-s*wheel_position[1];
      positions_[2*i+1]=steer_position[1]+s*wheel_position[0]+c*wheel_position[1];
    }
    if(bounds_.size()!=6)throw std::runtime_error("Incomplete arm joint limits");
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    arm_pub_=create_publisher<Array>("/arm_position_controller/commands",10);
    steer_pub_=create_publisher<Array>("/steer_position_controller/commands",10);
    wheel_pub_=create_publisher<Array>("/wheel_velocity_controller/commands",10);
    status_pub_=create_publisher<std_msgs::msg::String>("/simulation/control_status",10);
    feedback_sub_=create_subscription<Joints>("/joint_states",rclcpp::SensorDataQoS(),[this](Joints::SharedPtr m){std::lock_guard<std::mutex> lock(mutex_);receive_state(m);});
    arm_sub_=create_subscription<Joints>("/simulation/arm_command",10,[this](Joints::SharedPtr m){std::lock_guard<std::mutex> lock(mutex_);receive_arm(m);});
    base_sub_=create_subscription<Twist>("/simulation/cmd_vel",10,[this](Twist::SharedPtr m){std::lock_guard<std::mutex> lock(mutex_);receive_base(m);});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/ground_truth/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr m){std::lock_guard<std::mutex> lock(mutex_);receive_odom(m);});
    timer_=create_wall_timer(std::chrono::duration<double>(1./parameter("rate_hz")),[this]{std::lock_guard<std::mutex> lock(mutex_);tick();});
    RCLCPP_INFO(get_logger(),"C++ Gazebo actuator adapter ready; manual control and exclusive coupled plan ownership");
  }
};
SimControlHandle make_sim_control(const rclcpp::NodeOptions &options,const std::string &signature) {
  auto node=std::make_shared<SimControl>(options,signature);return {node,node};
}
}
