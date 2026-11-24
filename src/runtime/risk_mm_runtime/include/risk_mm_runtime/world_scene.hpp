#pragma once
#include <Eigen/Geometry>
#include <gazebo_msgs/msg/model_states.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace risk_mm {
struct WorldBox {
  std::string name;
  Eigen::Vector3d center, size;
};

// Immutable complete geometry of the selected local static world. Runtime
// freshness comes separately from checking Gazebo's actual model-state stream.
class WorldScene {
  std::vector<WorldBox> boxes_;
  std::map<std::string,Eigen::Isometry3d> models_;
  std::array<double,6> bounds_{};
  std::string signature_;
  double floor_z_{0.};
 public:
  WorldScene(const std::string &world_file,const std::string &metadata_file);
  const std::vector<WorldBox> &boxes() const {return boxes_;}
  const std::array<double,6> &bounds() const {return bounds_;}
  const std::string &signature() const {return signature_;}
  double floorHeight() const {return floor_z_;}
  unsigned long revision() const;
  bool matches(const gazebo_msgs::msg::ModelStates &,std::string &reason) const;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud(double resolution,
                                                const std_msgs::msg::Header &) const;
};
std::string content_sha256(const std::string &file);
}
