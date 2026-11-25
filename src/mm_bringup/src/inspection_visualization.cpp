#include <rclcpp/rclcpp.hpp>
#include <risk_mm_interfaces/msg/coupled_status.hpp>
#include <risk_mm_runtime/world_scene.hpp>
#include <risk_mm_runtime/task_goals.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>

namespace {
using Path=nav_msgs::msg::Path;
using Grid=nav_msgs::msg::OccupancyGrid;
using Marker=visualization_msgs::msg::Marker;
using Markers=visualization_msgs::msg::MarkerArray;
using Status=risk_mm_interfaces::msg::CoupledStatus;
geometry_msgs::msg::Point point(double x,double y,double z) {
  geometry_msgs::msg::Point p;p.x=x;p.y=y;p.z=z;return p;
}
double stamp(const std_msgs::msg::Header &h) {return rclcpp::Time(h.stamp).seconds();}

// Read-only rendering of the existing planner map and validated messages.
// This node has no motion publishers or planning/execution service clients.
class InspectionVisualization final : public rclcpp::Node {
  std::unique_ptr<risk_mm::WorldScene> world_;
  std::vector<risk_mm::TaskGoal> goals_;
  Grid global_;
  double window_,distance_range_,period_,last_cloud_ros_{-1.};
  size_t trace_limit_;
  Status::ConstSharedPtr status_;
  Path route_,trace_,active_;
  std::vector<Path> segments_;
  std::string plan_id_,mission_id_;
  std::chrono::steady_clock::time_point cloud_received_{},status_received_{};
  size_t visited_{0},total_{0};
  bool origin_valid_{false};std::array<double,9> origin_{};
  rclcpp::Publisher<Grid>::SharedPtr global_pub_,local_pub_;
  rclcpp::Publisher<Path>::SharedPtr route_pub_,trace_pub_,active_pub_;
  rclcpp::Publisher<Markers>::SharedPtr world_pub_,markers_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr patrol_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  template<class T>T parameter(const std::string &name,T fallback) {
    if(!has_parameter(name))declare_parameter<T>(name,fallback);
    return get_parameter(name).get_value<T>();
  }
  bool fresh() const {
    if(!status_ || !status_->map_ready || !status_->feedback_ready || last_cloud_ros_<0.)return false;
    const double age=now().seconds()-stamp(status_->header),map_age=now().seconds()-last_cloud_ros_;
    const auto wall=std::chrono::steady_clock::now();
    return age>=0. && age<=.5 && map_age>=0. && map_age<=2. &&
        std::chrono::duration<double>(wall-cloud_received_).count()<=2. &&
        std::chrono::duration<double>(wall-status_received_).count()<=2.;
  }
  void cloud(const sensor_msgs::msg::PointCloud2 &message) {
    if(message.header.frame_id!="world" || message.width==0)return;
    if(last_cloud_ros_>=0. && now().seconds()-last_cloud_ros_<period_)return;
    std::fill(global_.data.begin(),global_.data.end(),-1);
    // GridMap::publishESDF encodes signed distance [-3,3] m as intensity [0,1].
    // Invert that display encoding, then normalize distance for a heatmap.
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(message,"x"),y(message,"y"),intensity(message,"intensity");
      for(;x!=x.end();++x,++y,++intensity) {
        if(!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*intensity))continue;
        const int col=std::floor((*x-global_.info.origin.position.x)/global_.info.resolution);
        const int row=std::floor((*y-global_.info.origin.position.y)/global_.info.resolution);
        if(col<0 || row<0 || col>=int(global_.info.width) || row>=int(global_.info.height))continue;
        const double distance=6.*(*intensity)-3.;
        const int cost=distance<=0.?100:std::lround(99.*std::clamp(1.-distance/distance_range_,0.,1.));
        global_.data[row*global_.info.width+col]=static_cast<int8_t>(cost);
      }
      last_cloud_ros_=now().seconds();cloud_received_=std::chrono::steady_clock::now();
    }catch(const std::runtime_error &error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Cannot read planner ESDF: %s",error.what());
    }
  }
  void status(const Status::ConstSharedPtr &message) {
    if(status_ && stamp(message->header)<stamp(status_->header)) {
      segments_.clear();route_.poses.clear();trace_.poses.clear();origin_valid_=false;last_cloud_ros_=-1.;
    }
    if(message->plan_id!=plan_id_) {
      if(status_ || message->phase=="PLANNING") {segments_.clear();route_.poses.clear();active_.poses.clear();}
      plan_id_=message->plan_id;
      if(message->phase=="PLANNING") {
        trace_.poses.clear();origin_=message->state;origin_valid_=true;
      }
    }
    status_=message;status_received_=std::chrono::steady_clock::now();
  }
  void path(const Path &message) {
    if(message.header.frame_id!="world" || message.poses.empty())return;
    Path shown=message;
    for(auto &pose:shown.poses)pose.pose.position.z=.07;
    if(!segments_.empty()) {
      const auto &previous=segments_.back();
      if(stamp(previous.header)==stamp(shown.header))return;
      const auto &a=previous.poses.back().pose.position,&b=shown.poses.front().pose.position;
      if(std::hypot(a.x-b.x,a.y-b.y)>.03) {segments_.clear();route_.poses.clear();}
    }
    segments_.push_back(shown);
    route_.header=shown.header;
    route_.poses.insert(route_.poses.end(),shown.poses.begin(),shown.poses.end());
  }
  void odom(const nav_msgs::msg::Odometry &message) {
    if(message.header.frame_id!="world")return;
    geometry_msgs::msg::PoseStamped pose;pose.header=message.header;pose.pose=message.pose.pose;pose.pose.position.z=.11;
    if(!trace_.poses.empty()) {
      const auto &last=trace_.poses.back();
      if(stamp(pose.header)<stamp(last.header))trace_.poses.clear();
      else if(std::hypot(pose.pose.position.x-last.pose.position.x,pose.pose.position.y-last.pose.position.y)<.02 &&
              stamp(pose.header)-stamp(last.header)<1.)return;
    }
    trace_.header=message.header;trace_.poses.push_back(pose);
    if(trace_.poses.size()>trace_limit_)trace_.poses.erase(trace_.poses.begin());
  }
  Marker marker(const std::string &ns,int id,int type,double r,double g,double b,double alpha=1.) {
    Marker m;m.header.frame_id="world";m.header.stamp=now();m.ns=ns;m.id=id;m.type=type;
    m.action=Marker::ADD;m.pose.orientation.w=1.;m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=alpha;return m;
  }
  void markers() {
    Markers scene,mission;
    auto boxes=marker("world_collision_boxes",0,Marker::LINE_LIST,.60,.65,.72,.55);boxes.scale.x=.015;
    for(const auto &box:world_->boxes()) {
      const Eigen::Vector3d lo=box.center-box.size/2.,hi=box.center+box.size/2.;
      for(int corner=0;corner<8;++corner)for(int axis=0;axis<3;++axis)if(!(corner&(1<<axis))) {
        const int other=corner|(1<<axis);
        boxes.points.push_back(point(corner&1?hi.x():lo.x(),corner&2?hi.y():lo.y(),corner&4?hi.z():lo.z()));
        boxes.points.push_back(point(other&1?hi.x():lo.x(),other&2?hi.y():lo.y(),other&4?hi.z():lo.z()));
      }
    }
    scene.markers.push_back(boxes);world_pub_->publish(scene);
    const bool current_patrol=status_ && status_->plan_id==mission_id_;
    for(size_t i=0;i<goals_.size();++i) {
      const auto &goal=goals_[i];const bool done=current_patrol && i<visited_;
      auto pin=marker("inspection_points",i,Marker::SPHERE,done?.2:1.,done?1.:.65,.1);
      pin.pose.position=point(goal.input[0],goal.input[1],.12);pin.scale.x=pin.scale.y=.22;pin.scale.z=.16;mission.markers.push_back(pin);
      auto label=marker("point_labels",i,Marker::TEXT_VIEW_FACING,1.,1.,.9);
      label.pose.position=point(goal.input[0],goal.input[1],.5);label.scale.z=.18;label.text=std::to_string(i+1)+" "+goal.id;mission.markers.push_back(label);
      auto arrow=marker("point_heading",i,Marker::ARROW,1.,.65,.1);arrow.scale.x=.035;arrow.scale.y=.08;arrow.scale.z=.10;
      arrow.points={point(goal.input[0],goal.input[1],.14),point(goal.input[0]+.45*std::cos(goal.input[2]),goal.input[1]+.45*std::sin(goal.input[2]),.14)};
      mission.markers.push_back(arrow);
    }
    auto origin=marker("mission_origin",0,Marker::TEXT_VIEW_FACING,.3,1.,1.);origin.scale.z=.2;
    if(origin_valid_) {origin.pose.position=point(origin_[0],origin_[1],.55);origin.text="START / RETURN";}
    else origin.action=Marker::DELETE;
    mission.markers.push_back(origin);
    if(status_) {
      const double x=status_->state[0],y=status_->state[1];
      auto window=marker("local_window",0,Marker::LINE_STRIP,.2,.85,1.);window.scale.x=.025;
      const double h=window_/2.;window.points={point(x-h,y-h,.04),point(x+h,y-h,.04),point(x+h,y+h,.04),point(x-h,y+h,.04),point(x-h,y-h,.04)};
      mission.markers.push_back(window);
      auto label=marker("runtime_status",0,Marker::TEXT_VIEW_FACING,fresh()?.75:1.,fresh()?1.:.2,.25);label.scale.z=.17;
      label.pose.position=point(x,y,1.9);label.text=plan_id_+"  "+status_->phase+"\nMAP "+(fresh()?"CURRENT":"UNKNOWN / STALE");
      if(current_patrol)label.text+="\nVISITED "+std::to_string(visited_)+" / "+std::to_string(total_);
      mission.markers.push_back(label);
    }
    markers_pub_->publish(mission);
  }
  void publish() {
    Grid shown=global_;shown.header.stamp=now();
    if(!fresh())std::fill(shown.data.begin(),shown.data.end(),-1);
    global_pub_->publish(shown);
    if(status_) {
      Grid local;local.header=shown.header;local.info=shown.info;
      const int size=static_cast<int>(std::ceil(window_/shown.info.resolution));
      const int left=std::floor((status_->state[0]-window_/2.-shown.info.origin.position.x)/shown.info.resolution);
      const int bottom=std::floor((status_->state[1]-window_/2.-shown.info.origin.position.y)/shown.info.resolution);
      local.info.width=local.info.height=size;
      local.info.origin.position.x+=left*shown.info.resolution;local.info.origin.position.y+=bottom*shown.info.resolution;
      local.info.origin.position.z=.025;local.data.assign(size*size,-1);
      for(int y=0;y<size;++y)for(int x=0;x<size;++x) {
        const int gx=x+left,gy=y+bottom;
        if(gx>=0 && gy>=0 && gx<int(shown.info.width) && gy<int(shown.info.height))local.data[y*size+x]=shown.data[gy*shown.info.width+gx];
      }
      local_pub_->publish(local);
    }
    active_.poses.clear();active_.header.frame_id="world";active_.header.stamp=now();
    if(status_ && (status_->phase=="EXECUTING" || status_->phase=="ALIGNING" || status_->phase=="SETTLING"))
      for(const auto &segment:segments_) {
        const auto &end=segment.poses.back().pose.position;
        if(std::hypot(end.x-status_->goal[0],end.y-status_->goal[1])<.01){active_=segment;break;}
      }
    for(auto *path:{&route_,&trace_}){path->header.frame_id="world";path->header.stamp=now();}
    route_pub_->publish(route_);trace_pub_->publish(trace_);active_pub_->publish(active_);markers();
  }
 public:
  InspectionVisualization():Node("inspection_visualization",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    const auto world=parameter<std::string>("world_file",""),metadata=parameter<std::string>("world_metadata_file","");
    world_=std::make_unique<risk_mm::WorldScene>(world,metadata);
    goals_=risk_mm::loadTaskGoals(parameter<std::string>("patrol_file",""),std::filesystem::path(world).stem().string(),world_->signature());
    const double resolution=parameter<double>("map_resolution",0.);
    window_=parameter<double>("local_window_m",6.);distance_range_=parameter<double>("distance_range_m",1.);
    const double rate=parameter<double>("publish_hz",2.);const auto limit=parameter<int64_t>("max_trace_points",10000);
    if(!std::isfinite(resolution) || resolution<=0. || !std::isfinite(window_) || window_<=0. || !std::isfinite(distance_range_) || distance_range_<=0. || !std::isfinite(rate) || rate<=0. || rate>10. || limit<2)
      throw std::runtime_error("invalid visualization dimensions/rate");
    period_=1./rate;trace_limit_=limit;
    const auto &bounds=world_->bounds();global_.header.frame_id="world";global_.info.resolution=resolution;
    global_.info.width=std::lround((bounds[1]-bounds[0])/resolution);global_.info.height=std::lround((bounds[3]-bounds[2])/resolution);
    global_.info.origin.position=point(bounds[0],bounds[2],.01);global_.info.origin.orientation.w=1.;global_.data.assign(global_.info.width*global_.info.height,-1);
    const auto latched=rclcpp::QoS(1).transient_local();
    global_pub_=create_publisher<Grid>("/inspection/global_costmap",latched);local_pub_=create_publisher<Grid>("/inspection/local_costmap",latched);
    route_pub_=create_publisher<Path>("/inspection/validated_route",latched);trace_pub_=create_publisher<Path>("/inspection/measured_path",latched);active_pub_=create_publisher<Path>("/inspection/active_path",latched);
    world_pub_=create_publisher<Markers>("/inspection/world_markers",latched);markers_pub_=create_publisher<Markers>("/inspection/task_markers",latched);
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/grid_map/esdf",rclcpp::QoS(1),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){cloud(*m);});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/ground_truth/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odom(*m);});
    status_sub_=create_subscription<Status>("/coupled/status",10,[this](Status::ConstSharedPtr m){status(m);});
    path_sub_=create_subscription<Path>("/coupled/validated_path",latched,[this](Path::ConstSharedPtr m){path(*m);});
    patrol_sub_=create_subscription<std_msgs::msg::String>("/coupled/patrol_status",10,[this](std_msgs::msg::String::ConstSharedPtr m){
      const auto value=YAML::Load(m->data);mission_id_=value["mission_id"].as<std::string>();visited_=value["visited"].as<size_t>();total_=value["total"].as<size_t>();
    });
    timer_=create_wall_timer(std::chrono::duration<double>(period_),[this]{publish();});
    RCLCPP_INFO(get_logger(),"Read-only RViz bridge: actual planner ESDF heatmap, local crop, validated/active/measured paths; no motion interfaces");
  }
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<InspectionVisualization>());}
  catch(const std::exception &error){RCLCPP_ERROR(rclcpp::get_logger("inspection_visualization"),"%s",error.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
