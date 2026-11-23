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
#include <optional>
#include <sstream>

namespace risk_mm {
class SimControl final : public rclcpp::Node {
  using Joints=sensor_msgs::msg::JointState;
  using Twist=geometry_msgs::msg::TwistStamped;
  using Array=std_msgs::msg::Float64MultiArray;
  std::vector<std::string> arm_,steer_,wheels_;
  std::vector<std::pair<double,double>> bounds_;
  std::array<double,8> positions_{};
  std::array<double,4> steer_applied_{},wheel_applied_{};
  std::vector<double> arm_applied_;
  Joints::SharedPtr feedback_,target_;
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
  double parameter(const char *key) const {return config_[key].as<double>();}
  bool fresh(const std_msgs::msg::Header &header,double age) {
    double delta=now().seconds()-rclcpp::Time(header.stamp).seconds();return delta>=0. && delta<=age;
  }
  static double approach(double current,double goal,double step) {return current+std::clamp(goal-current,-step,step);}
  static Array array(const std::vector<double> &values) {Array message;message.data=values;return message;}
  template<size_t N> static Array array(const std::array<double,N> &values) {
    return array(std::vector<double>(values.begin(),values.end()));
  }
  void receive_state(const Joints::SharedPtr message) {
    std::map<std::string,size_t> index;
    for(size_t i=0;i<message->name.size();++i)index.emplace(message->name[i],i);
    bool valid=index.size()==message->name.size() && fresh(message->header,parameter("feedback_timeout_s"));
    for(const auto &group:{arm_,steer_,wheels_})for(const auto &name:group) {
      if(!index.count(name)){valid=false;continue;}
      auto i=index[name];valid=valid && i<message->position.size() && i<message->velocity.size();
      if(i<message->position.size() && i<message->velocity.size())
        valid=valid && std::isfinite(message->position[i]) && std::isfinite(message->velocity[i]);
    }
    if(!valid){feedback_.reset();return;}
    feedback_=message;
    if(arm_applied_.empty()) {
      for(auto &name:arm_)arm_applied_.push_back(message->position[index[name]]);
      for(size_t i=0;i<4;++i)steer_applied_[i]=message->position[index[steer_[i]]];
    }
  }
  void receive_arm(const Joints::SharedPtr message) {
    target_.reset();
    if(!fresh(message->header,parameter("command_timeout_s")) || message->name.size()!=6 || message->position.size()!=6)return;
    std::map<std::string,double> values;
    for(size_t i=0;i<6;++i)values.emplace(message->name[i],message->position[i]);
    if(values.size()!=6)return;
    auto ordered=std::make_shared<Joints>();ordered->header=message->header;ordered->name=arm_;
    for(size_t i=0;i<6;++i) {
      if(!values.count(arm_[i]))return;
      double v=values.at(arm_[i]);if(!std::isfinite(v) || v<bounds_[i].first || v>bounds_[i].second)return;
      ordered->position.push_back(v);
    }target_=ordered;
  }
  void receive_base(const Twist::SharedPtr message) {
    base_.reset();auto &t=message->twist;
    if(message->header.frame_id!="base_link" || !fresh(message->header,parameter("command_timeout_s")))return;
    for(double value:{t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z})if(!std::isfinite(value))return;
    if(std::abs(t.linear.z)+std::abs(t.angular.x)+std::abs(t.angular.y)>1e-9 ||
       std::hypot(t.linear.x,t.linear.y)>parameter("base_speed_max_m_s") ||
       std::abs(t.angular.z)>parameter("base_yaw_max_rad_s"))return;
    base_=message;
  }
  void receive_odom(const nav_msgs::msg::Odometry::SharedPtr message) {
    if(!fresh(message->header,parameter("feedback_timeout_s")))return;
    auto &p=message->pose.pose.position;auto &q=message->pose.pose.orientation;
    for(double value:{p.x,p.y,p.z,q.x,q.y,q.z,q.w})if(!std::isfinite(value))return;
    geometry_msgs::msg::TransformStamped t;t.header=message->header;t.header.frame_id="world";t.child_frame_id="base_link";
    t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z;t.transform.rotation=q;tf_->sendTransform(t);
  }
  void tick() {
    const double stamp=now().seconds();double dt=last_tick_<0?.02:std::clamp(stamp-last_tick_,0.,.05);
    if(last_tick_>=0 && stamp<last_tick_){feedback_.reset();base_.reset();target_.reset();last_status_=-1.;}
    last_tick_=stamp;
    const bool ready=feedback_ && fresh(feedback_->header,parameter("feedback_timeout_s"));
    const bool arm_active=ready && target_ && fresh(target_->header,parameter("command_timeout_s"));
    const bool base_active=ready && base_ && fresh(base_->header,parameter("command_timeout_s"));
    if(ready) {
      std::map<std::string,double> measured;for(size_t i=0;i<feedback_->name.size();++i)measured[feedback_->name[i]]=feedback_->position[i];
      for(size_t i=0;i<6;++i)arm_applied_[i]=approach(arm_applied_[i],arm_active?target_->position[i]:measured[arm_[i]],parameter("arm_rate_rad_s")*dt);
      auto steer_goal=steer_applied_;std::array<double,4> wheel_goal{};
      if(base_active) {
        const auto &t=base_->twist;
        const auto solution=mm_model::inverse(t.linear.x,t.linear.y,t.angular.z,positions_,wheel_radius_,
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
        <<",\"base_command_active\":"<<base_active<<",\"reason\":\""<<(ready?"manual Gazebo control":"missing/stale joint feedback")<<"\",\"wheel_applied\":[";
      for(size_t i=0;i<4;++i)out<<(i?",":"")<<wheel_applied_[i];out<<"]}";
      std_msgs::msg::String status;status.data=out.str();status_pub_->publish(status);last_status_=stamp;
    }
  }
public:
  SimControl():Node("sim_control") {
    auto share=[](const std::string &p){return ament_index_cpp::get_package_share_directory(p);};
    config_=YAML::LoadFile(share("mm_bringup")+"/config/manual_control.yaml");
    const auto collision=YAML::LoadFile(share("mm_bringup")+"/config/simulation_collision.yaml");wheel_radius_=collision["tire"]["radius"].as<double>();
    tinyxml2::XMLDocument doc;
    if(doc.LoadFile((share("rangermini_description")+"/urdf/rangermini_xarm6.urdf").c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("Missing robot URDF");
    for(int i=1;i<=6;++i)arm_.push_back("xarm6_joint"+std::to_string(i));
    for(auto *w:{"fl","fr","rl","rr"}){steer_.push_back(std::string("joint_steer_")+w);wheels_.push_back(std::string("joint_wheel_")+w);}
    for(const auto &name:arm_)for(auto *j=doc.RootElement()->FirstChildElement("joint");j;j=j->NextSiblingElement("joint"))if(name==j->Attribute("name")) {
      auto *l=j->FirstChildElement("limit");bounds_.emplace_back(l->DoubleAttribute("lower"),l->DoubleAttribute("upper"));
    }
    for(size_t i=0;i<4;++i) {
      double x=0.,y=0.,z=0.;
      for(const auto &name:{steer_[i],wheels_[i]})for(auto *j=doc.RootElement()->FirstChildElement("joint");j;j=j->NextSiblingElement("joint"))if(name==j->Attribute("name")) {
        double a,b,c;std::istringstream(j->FirstChildElement("origin")->Attribute("xyz"))>>a>>b>>c;x+=a;y+=b;z+=c;
      }positions_[2*i]=x;positions_[2*i+1]=y;
    }
    if(bounds_.size()!=6)throw std::runtime_error("Incomplete arm joint limits");
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    arm_pub_=create_publisher<Array>("/arm_position_controller/commands",10);
    steer_pub_=create_publisher<Array>("/steer_position_controller/commands",10);
    wheel_pub_=create_publisher<Array>("/wheel_velocity_controller/commands",10);
    status_pub_=create_publisher<std_msgs::msg::String>("/simulation/control_status",10);
    feedback_sub_=create_subscription<Joints>("/joint_states",rclcpp::SensorDataQoS(),[this](Joints::SharedPtr m){receive_state(m);});
    arm_sub_=create_subscription<Joints>("/simulation/arm_command",10,[this](Joints::SharedPtr m){receive_arm(m);});
    base_sub_=create_subscription<Twist>("/simulation/cmd_vel",10,[this](Twist::SharedPtr m){receive_base(m);});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/ground_truth/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr m){receive_odom(m);});
    timer_=create_wall_timer(std::chrono::duration<double>(1./parameter("rate_hz")),[this]{tick();});
    RCLCPP_INFO(get_logger(),"C++ manual Gazebo controls ready; no planner trajectory execution enabled");
  }
};
}

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<risk_mm::SimControl>());}
  catch(const std::exception &e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();
}
