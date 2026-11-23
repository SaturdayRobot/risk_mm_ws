#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "mm_config/mm_config.hpp"

#include <limits>
#include <algorithm>
#include <array>
#include <numeric>
#include <pcl/kdtree/kdtree_flann.h>

namespace remani_planner {
struct MMConfig::BodyIndex {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  // Exact double-precision, unsorted radius queries for cost accumulation.
  // Validation keeps its nearest-first PCL query and existing public behavior.
  struct RadiusNode {
    Eigen::Vector3d minimum, maximum;
    int begin, end, left{-1}, right{-1};
  };
  std::vector<Eigen::Vector3d> exact_points;
  std::vector<int> order;
  std::vector<RadiusNode> nodes;
  int buildRadiusNode(int begin, int end) {
    RadiusNode node;node.begin=begin;node.end=end;
    node.minimum=node.maximum=exact_points[order[begin]];
    for(int i=begin+1;i<end;++i) {
      node.minimum=node.minimum.cwiseMin(exact_points[order[i]]);
      node.maximum=node.maximum.cwiseMax(exact_points[order[i]]);
    }
    const int id=nodes.size();nodes.push_back(node);
    if(end-begin>12) {
      Eigen::Index axis;(node.maximum-node.minimum).maxCoeff(&axis);
      const int middle=(begin+end)/2;
      std::nth_element(order.begin()+begin,order.begin()+middle,order.begin()+end,
        [&](int a,int b){return exact_points[a][axis]<exact_points[b][axis];});
      const int left=buildRadiusNode(begin,middle),right=buildRadiusNode(middle,end);
      nodes[id].left=left;nodes[id].right=right;
    }
    return id;
  }
  void buildRadiusIndex() {
    order.resize(exact_points.size());std::iota(order.begin(),order.end(),0);
    nodes.clear();nodes.reserve(2*order.size());
    if(!order.empty())buildRadiusNode(0,order.size());
  }
  void queryRadiusNode(int id,const Eigen::Vector3d &point,double squared,
                       std::vector<int> &result) const {
    const auto &node=nodes[id];
    const Eigen::Vector3d delta=(node.minimum-point).cwiseMax(0.)+
                               (point-node.maximum).cwiseMax(0.);
    if(delta.squaredNorm()>squared)return;
    if(node.left<0) {
      for(int i=node.begin;i<node.end;++i)
        if((exact_points[order[i]]-point).squaredNorm()<=squared)
          result.push_back(order[i]);
    } else {
      queryRadiusNode(node.left,point,squared,result);
      queryRadiusNode(node.right,point,squared,result);
    }
  }
  std::vector<int> unsortedNeighbors(const Eigen::Vector3d &point,double radius) const {
    std::vector<int> result;
    if(!nodes.empty())queryRadiusNode(0,point,radius*radius,result);
    return result;
  }
  bool obstacleHit(int id,const Eigen::Matrix4d &transform,GridMap &map,
                   double threshold,double floor) const {
    const auto &node=nodes[id];
    const auto rotation=transform.topLeftCorner<3,3>();
    const auto translation=transform.topRightCorner<3,1>();
    const Eigen::Vector3d center=rotation*((node.minimum+node.maximum)*.5)+translation;
    const Eigen::Vector3d extent=rotation.cwiseAbs()*((node.maximum-node.minimum)*.5);
    if(center.z()-extent.z()>floor+1e-9 &&
       map.interpolationBoxClear(center-extent,center+extent,threshold))return false;
    if(node.left>=0)return obstacleHit(node.left,transform,map,threshold,floor)||
                           obstacleHit(node.right,transform,map,threshold,floor);
    for(int i=node.begin;i<node.end;++i) {
      const Eigen::Vector3d p=rotation*exact_points[order[i]]+translation;
      if(p.z()<floor || map.getPreciseDistance(p)<threshold)return true;
    }
    return false;
  }
  template<typename Allowed>
  bool anyWithin(int id,const Eigen::Vector3d &point,double squared,const Allowed &allowed) const {
    const auto &node=nodes[id];
    const Eigen::Vector3d delta=(node.minimum-point).cwiseMax(0.)+(point-node.maximum).cwiseMax(0.);
    if(delta.squaredNorm()>squared+1e-12)return false;
    if(node.left>=0)return anyWithin(node.left,point,squared,allowed)||anyWithin(node.right,point,squared,allowed);
    for(int i=node.begin;i<node.end;++i) {
      const int index=order[i];
      if(allowed(index) && (exact_points[index]-point).squaredNorm()<squared)return true;
    }
    return false;
  }
  std::vector<double> owners;
  Eigen::Vector3d minimum, maximum;
  bool outside(const Eigen::Vector3d &point, double radius) const {
    const Eigen::Vector3d delta = (minimum-point).cwiseMax(0.) + (point-maximum).cwiseMax(0.);
    return delta.squaredNorm() > (radius+1e-5)*(radius+1e-5);
  }
};
namespace {
struct TransformedBox {
  Eigen::Vector3d minimum, maximum;
};
TransformedBox transformedBox(const Eigen::Vector3d &minimum, const Eigen::Vector3d &maximum,
                              const Eigen::Matrix4d &transform) {
  const Eigen::Vector3d center=transform.topLeftCorner<3,3>()*((minimum+maximum)*.5)+transform.topRightCorner<3,1>();
  const Eigen::Vector3d extent=transform.topLeftCorner<3,3>().cwiseAbs()*((maximum-minimum)*.5);
  return {center-extent,center+extent};
}
}
std::vector<Eigen::Vector3d> MMConfig::baseNeighbors(const Eigen::Vector3d &point,double radius,int arm_link,bool sorted) const {
  if (body_index_->outside(point,radius)) return {};
  std::vector<int> indices;std::vector<float> squared;
  if(sorted)body_index_->tree.radiusSearch(pcl::PointXYZ(point.x(),point.y(),point.z()),radius+1e-5,indices,squared);
  else indices=body_index_->unsortedNeighbors(point,radius+1e-5);
  std::vector<Eigen::Vector3d> result;result.reserve(indices.size());
  for(int index:indices) {
    if(arm_link<=1 && body_index_->owners[index]==1.0)continue;
    if ((base_collision_points_[index] - point).squaredNorm() < radius * radius)
      result.push_back(base_collision_points_[index]);
  }
  return result;
}

std::vector<int> MMConfig::armNeighbors(int link,const Eigen::Vector3d &point,double radius,bool sorted) const {
  if (arm_indices_.at(link)->outside(point,radius)) return {};
  std::vector<int> indices;std::vector<float> squared;
  // Index uses float coordinates; pad only the broad phase, then callers
  // retain the original double-precision distance threshold.
  if(sorted)arm_indices_.at(link)->tree.radiusSearch(pcl::PointXYZ(point.x(),point.y(),point.z()),radius+1e-5,indices,squared);
  else indices=arm_indices_.at(link)->unsortedNeighbors(point,radius+1e-5);
  return indices;
}


void MMConfig::setParam(rclcpp::Node::SharedPtr &nh,
                        const std::shared_ptr<GridMap> &env) {
  grid_map_ = env;
  setParam(nh);
}

void MMConfig::setParam(rclcpp::Node::SharedPtr &nh) {
  node_=nh;

  nh->get_parameter_or("mm.mobile_base_dof", mobile_base_dof_, 2);
  nh->get_parameter_or("mm.manipulator_dof", manipulator_dof_, 6);
  if (mobile_base_dof_ != 2 || manipulator_dof_ != 6)
    throw std::runtime_error("REMANI state must be [x,y,q1..q6]; yaw follows the path tangent");
  nh->get_parameter_or("mm.mobile_base_length", mobile_base_length_, 0.752);
  nh->get_parameter_or("mm.mobile_base_width", mobile_base_width_, 0.512);
  nh->get_parameter_or("mm.mobile_base_height", mobile_base_height_, 0.836);
  nh->get_parameter_or("mm.mobile_base_check_radius", mobile_base_check_radius_, 0.085);
  nh->get_parameter_or("mm.mobile_base_wheel_base", mobile_base_wheel_base_, 0.494003);
  nh->get_parameter_or("mm.mobile_base_wheel_radius", mobile_base_wheel_radius_, 0.1);
  nh->get_parameter_or("mm.mobile_base_non_singul_vel", mobile_base_non_singul_vel_, 0.02);
  nh->get_parameter_or("mm.mobile_base_max_wheel_omega", mobile_base_max_wheel_omega_, 3.5);
  nh->get_parameter_or("mm.mobile_base_max_wheel_alpha", mobile_base_max_wheel_alpha_, 5.0);
  mobile_base_max_vel_ = mobile_base_max_wheel_omega_ * mobile_base_wheel_radius_;
  mobile_base_max_acc_ = mobile_base_max_wheel_alpha_ * mobile_base_wheel_radius_;
  nh->get_parameter_or("mm.manipulator_thickness", manipulator_thickness_, 0.055);
  nh->get_parameter_or("grid_map.resolution", map_resolution_, 0.1);
  nh->get_parameter_or("optimization.safe_margin", car_safe_margin_, 0.03);
  nh->get_parameter_or("optimization.safe_margin_mani", mani_safe_margin_, 0.02);
  nh->get_parameter_or("optimization.self_safe_margin", self_safe_margin_, 0.015);
  nh->get_parameter_or("optimization.ground_safe_dis", ground_safe_dis_, 0.03);
  auto required = [&](const std::string &key, size_t count) {
    std::vector<double> value;
    if (!nh->get_parameter(key, value) || value.size() != count)
      throw std::runtime_error("Missing/invalid URDF-derived parameter " + key);
    return value;
  };
  auto mount = required("mm.arm_mount_transform", 16);
  auto origins = required("mm.arm_joint_origins", 96);
  auto axes = required("mm.arm_joint_axes", 18);
  auto lower = required("mm.manipulator_min_pos", 6);
  auto upper = required("mm.manipulator_max_pos", 6);
  for (int r=0; r<4; ++r) for (int c=0; c<4; ++c) T_q_0_(r,c)=mount[4*r+c];
  manipulator_min_pos_ = Eigen::Map<Eigen::VectorXd>(lower.data(), 6);
  manipulator_max_pos_ = Eigen::Map<Eigen::VectorXd>(upper.data(), 6);
  manipulator_config_ = Eigen::VectorXd::Zero(6);
  joint_origins_.resize(6); joint_axes_.resize(6); manipulator_link_pts_.clear();
  collision_owners_.clear();arm_indices_.clear();
  std::vector<double> disabled;
  if(!nh->get_parameter("mm.self_collision_disabled_pairs", disabled) || disabled.size()%2)
    throw std::runtime_error("Missing collision semantics");
  for(size_t k=0;k<disabled.size();k+=2) {
    int a=static_cast<int>(disabled[k]),b=static_cast<int>(disabled[k+1]);
    if(a<0||a>6||b<0||b>6)throw std::runtime_error("Invalid collision pair");
    disabled_self_pairs_[a][b]=disabled_self_pairs_[b][a]=true;
  }
  for (int i=0; i<6; ++i) {
    for (int r=0; r<4; ++r) for (int c=0; c<4; ++c)
      joint_origins_[i](r,c)=origins[16*i+4*r+c];
    joint_axes_[i] = Eigen::Vector3d(axes[3*i], axes[3*i+1], axes[3*i+2]).normalized();
    std::vector<double> points;
    if (!nh->get_parameter("mm.arm_collision_points_"+std::to_string(i), points) || points.empty() || points.size()%3)
      throw std::runtime_error("Missing arm collision samples");
    Eigen::Matrix4Xd cloud(4, points.size()/3);
    for (size_t k=0; k<points.size()/3; ++k)
      cloud.col(k) << points[3*k],points[3*k+1],points[3*k+2],1.0;
    manipulator_link_pts_.push_back(cloud);
    auto index=std::make_shared<BodyIndex>();
    for(int k=0;k<cloud.cols();++k)index->points->push_back(pcl::PointXYZ(cloud(0,k),cloud(1,k),cloud(2,k)));
    index->tree.setInputCloud(index->points);
    for(int k=0;k<cloud.cols();++k)index->exact_points.push_back(cloud.col(k).head<3>());
    index->buildRadiusIndex();
    index->minimum=cloud.topRows<3>().rowwise().minCoeff();
    index->maximum=cloud.topRows<3>().rowwise().maxCoeff();
    arm_indices_.push_back(index);
    auto owners=required("mm.arm_collision_owners_"+std::to_string(i),cloud.cols());
    std::vector<int> ids; for(double owner:owners)ids.push_back(static_cast<int>(owner));
    collision_owners_.push_back(ids);
  }
  // Cache owner-aware exclusions, including camera geometry on the last link.
  for (int x=0; x<7; ++x) for (int b=0; b<6; ++b)
    for (int y : collision_owners_[b])
      if (x != y && !disabled_self_pairs_[x][y]) active_owner_link_pairs_[x][b] = true;
  for (int a=0; a<6; ++a) for (int b=0; b<6; ++b)
    for (int x : collision_owners_[a])
      if (active_owner_link_pairs_[x][b]) active_self_link_pairs_[a][b] = true;
  body_index_=std::make_shared<BodyIndex>();
  std::vector<double> base_points;
  if (!nh->get_parameter("mm.base_collision_points", base_points) || base_points.empty() || base_points.size()%3)
    throw std::runtime_error("Missing base collision samples");
  base_collision_points_.clear();
  for (size_t i=0; i<base_points.size(); i+=3)
    base_collision_points_.emplace_back(base_points[i],base_points[i+1],base_points[i+2]);
  for(const auto &point:base_collision_points_)
    body_index_->points->push_back(pcl::PointXYZ(point.x(),point.y(),point.z()));
  body_index_->tree.setInputCloud(body_index_->points);
  body_index_->exact_points=base_collision_points_;
  body_index_->buildRadiusIndex();
  body_index_->minimum=base_collision_points_.front();
  body_index_->maximum=base_collision_points_.front();
  for (const auto &point : base_collision_points_) {
    body_index_->minimum=body_index_->minimum.cwiseMin(point);
    body_index_->maximum=body_index_->maximum.cwiseMax(point);
  }
  body_index_->owners=required("mm.base_collision_owners",base_collision_points_.size());
  nh->get_parameter("mm.arm_mesh_resources", arm_mesh_resources_);
  nh->get_parameter("mm.base_mesh_resource", mesh_resource_mobile_base_);
  B_h_ << 0.0,-1.0,1.0,0.0;
  vis_idx_size_=100; setColorSet();
  RCLCPP_INFO(rclcpp::get_logger("mm_config"), "[MMConfig] URDF chain loaded: 6 joints, base collision samples=%zu", base_collision_points_.size());
}

void MMConfig::setColorSet() {
  color_set_.clear();
  // red
  Eigen::Vector3d color;
  color << 255, 31, 91;
  color /= 255.0;
  color_set_.push_back(color);
  // green
  color << 0, 205, 108;
  color /= 255.0;
  color_set_.push_back(color);
  // blue
  color << 0, 154, 222;
  color /= 255.0;
  color_set_.push_back(color);
  // yellow
  color << 255, 198, 30;
  color /= 255.0;
  color_set_.push_back(color);
  // grey
  color << 160, 177, 186;
  color /= 255.0;
  color_set_.push_back(color);
  // orange
  color << 234, 96, 22;
  color /= 255.0;
  color_set_.push_back(color);
  // purple
  color << 175, 88, 186;
  color /= 255.0;
  color_set_.push_back(color);
  // brown
  color << 166, 118, 29;
  color /= 255.0;
  color_set_.push_back(color);
}

void MMConfig::getAJointTran(int joint_num, double theta, Eigen::Matrix4d &T,
                             Eigen::Matrix4d &T_grad) const {
  const auto &axis = joint_axes_.at(joint_num);
  Eigen::Matrix3d skew;
  skew << 0,-axis.z(),axis.y(), axis.z(),0,-axis.x(), -axis.y(),axis.x(),0;
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(theta,axis).toRotationMatrix();
  T = joint_origins_.at(joint_num);
  T.block<3,3>(0,0) = joint_origins_[joint_num].block<3,3>(0,0)*rotation;
  T_grad = Eigen::Matrix4d::Zero();
  T_grad.block<3,3>(0,0) = T.block<3,3>(0,0)*skew;
}

void MMConfig::visMM(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, std::string ns, int idx, double alpha,
                     const Eigen::Vector3d &car_state,
                     const Eigen::VectorXd &joint_state,
                     const bool &gripper_close) {
  if (pub->get_subscription_count() < 1)
    return;
  visualization_msgs::msg::MarkerArray marker_array;
  getMMMarkerArray(marker_array, ns, idx, alpha, car_state, joint_state,
                   gripper_close);
  pub->publish(marker_array);
}

void MMConfig::visMMCheckBall(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx,
                              double alpha, const Eigen::Vector3d &car_state,
                              const Eigen::VectorXd &joint_state) {
  if (pub->get_subscription_count() < 1)
    return;
  visCarCheckBall(pub, ns, idx, alpha, car_state);
  visManiCheckBall(pub, ns, idx, alpha, car_state, joint_state);
}

void MMConfig::getMMMarkerArray(visualization_msgs::msg::MarkerArray &marker_array,
                                std::string ns, int idx, double alpha,
                                const Eigen::Vector3d &car_state,
                                const Eigen::VectorXd &joint_state,
                                const bool &gripper_close) {
  marker_array.markers.clear();
  visualization_msgs::msg::MarkerArray car_marker_array =
      getCarMarkerArray(ns, idx, alpha, car_state);
  marker_array.markers.insert(marker_array.markers.end(),
                              car_marker_array.markers.begin(),
                              car_marker_array.markers.end());
  ;
  visualization_msgs::msg::MarkerArray mani_marker_array =
      getManiMarkerArray(ns, idx, alpha, car_state, joint_state, gripper_close);
  marker_array.markers.insert(marker_array.markers.end(),
                              mani_marker_array.markers.begin(),
                              mani_marker_array.markers.end());
  ;
}

double MMConfig::calYaw(const Eigen::Vector2d &vel_car, int traj_dir) {
  const Eigen::Vector2d vel_reg = regularizeBaseVelocity(vel_car, traj_dir);
  return atan2(traj_dir * vel_reg(1), traj_dir * vel_reg(0));
}

Eigen::Matrix2d MMConfig::calR(const Eigen::Vector2d &vel_car, int traj_dir) {
  double yaw = calYaw(vel_car, traj_dir);
  Eigen::Matrix2d R;
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  // R = R * traj_dir / vel_car.norm();
  return R;
}

Eigen::Vector2d MMConfig::regularizeBaseVelocity(const Eigen::Vector2d &vel_car,
                                                 int traj_dir) const {
  const double v_eps = std::max(1.0e-3, mobile_base_non_singul_vel_);
  if (vel_car.norm() >= v_eps) {
    return vel_car;
  }
  Eigen::Vector2d dir = vel_car;
  if (dir.norm() < 1.0e-6) {
    dir = Eigen::Vector2d(static_cast<double>(traj_dir), 0.0);
  } else {
    dir.normalize();
  }
  return dir * v_eps;
}

Eigen::Matrix2d MMConfig::caldRldv(const Eigen::Vector2d &vel_car,
                                   const Eigen::Vector2d &delta_p,
                                   int traj_dir) {
  const Eigen::Vector2d vel_reg = regularizeBaseVelocity(vel_car, traj_dir);
  const double v_norm = std::max(vel_reg.norm(), std::max(1.0e-3, mobile_base_non_singul_vel_));
  const double v_sq = std::max(vel_reg.squaredNorm(), v_norm * v_norm);
  Eigen::Matrix2d dRldv;
  dRldv << delta_p, B_h_ * delta_p;
  dRldv = dRldv * traj_dir / v_norm -
          calR(vel_reg, traj_dir) * delta_p * vel_reg.transpose() / v_sq;
  return dRldv;
}

Eigen::Vector2d MMConfig::caldYawdV(const Eigen::Vector2d &vel_car) {
  const Eigen::Vector2d vel_reg = regularizeBaseVelocity(vel_car, 1);
  const double v_sq = std::max(vel_reg.squaredNorm(),
                               std::pow(std::max(1.0e-3, mobile_base_non_singul_vel_), 2));
  Eigen::Vector2d dYawdV;
  dYawdV = B_h_ * vel_reg / v_sq;
  return dYawdV;
}

void MMConfig::getCarPts(const Eigen::Vector3d &car_state,
                         std::vector<Eigen::Vector3d> &car_pts) {
  getCarPts(car_state, car_pts, Eigen::Vector3d(0, 0, 0));
}

void MMConfig::getCarPts(const Eigen::Vector3d &car_state,
                         std::vector<Eigen::Vector3d> &car_pts,
                         const Eigen::Vector3d &inflate_size) {
  car_pts.clear();
  const Eigen::Matrix2d rotation = Eigen::Rotation2Dd(car_state.z()).toRotationMatrix();
  for (const auto &point:base_collision_points_) {
    Eigen::Vector3d p=point;
    p.head<2>()=rotation*point.head<2>()+car_state.head<2>();
    car_pts.push_back(p);
  }

}

void MMConfig::getCarPtsGrad(const Eigen::Vector2d &pos_car,
                             const Eigen::Vector2d &vel_car, const int traj_dir,
                             const Eigen::Vector3d &inflate_size,
                             std::vector<Eigen::Vector3d> &car_pts,
                             std::vector<Eigen::Matrix2d> &dPtdv) {
  car_pts.clear(); dPtdv.clear();
  const Eigen::Matrix2d rotation=calR(vel_car,traj_dir);
  for(const auto &point:base_collision_points_) {
    Eigen::Vector3d p=point; p.head<2>()=pos_car+rotation*point.head<2>();
    car_pts.push_back(p); dPtdv.push_back(caldRldv(vel_car,point.head<2>(),traj_dir));
  }

}

void MMConfig::getCarPtsGradNew(const Eigen::Vector2d &pos_car,
                                const Eigen::Vector2d &vel_car,
                                const int traj_dir,
                                const Eigen::Vector3d &inflate_size,
                                std::vector<Eigen::Vector3d> &car_pts,
                                std::vector<Eigen::Vector2d> &dPtdYaw) {
  car_pts.clear(); dPtdYaw.clear();
  const Eigen::Matrix2d rotation=calR(vel_car,traj_dir);
  for(const auto &point:base_collision_points_) {
    Eigen::Vector3d p=point; p.head<2>()=pos_car+rotation*point.head<2>();
    car_pts.push_back(p); dPtdYaw.push_back(rotation*B_h_*point.head<2>());
  }

}

void MMConfig::CarState2T(const Eigen::Vector3d &car_state,
                          Eigen::Matrix4d &T_car) {
  T_car.setIdentity();
  double c = cos(car_state(2)), s = sin(car_state(2));
  Eigen::Matrix3d R;
  R << c, -s, 0, s, c, 0, 0, 0, 1;
  T_car.block(0, 3, 3, 1) = Eigen::Vector3d(car_state(0), car_state(1), 0.0);
  T_car.block(0, 0, 3, 3) = R;
}

void MMConfig::getJointTMat(const Eigen::VectorXd &theta,
                            std::vector<Eigen::Matrix4d> &T_joint) {
  T_joint.clear();
  Eigen::Matrix4d T_temp, T_temp_grad_nouse;
  T_temp = Eigen::Matrix4d::Identity();
  T_joint.push_back(T_temp);
  for (int i = 0; i < manipulator_dof_ - 1; ++i) {
    getAJointTran(i, theta(i), T_temp, T_temp_grad_nouse);
    T_joint.push_back(T_temp);
  }
}

bool MMConfig::checkCarObsCollision(Eigen::Vector3d car_state, bool precise,
                                    bool safe, double &min_dist) {
  const double safe_dist=(safe?mobile_base_check_radius_+car_safe_margin_:mobile_base_check_radius_)+map_resolution_;
  Eigen::Matrix4d transform;
  CarState2T(car_state,transform);
  const auto box=transformedBox(body_index_->minimum,body_index_->maximum,transform);
  if(grid_map_->interpolationBoxClear(box.minimum,box.maximum,safe_dist)) {
    min_dist=safe_dist;
    return false;
  }
  std::vector<Eigen::Vector3d> car_pts;
  getCarPts(car_state,car_pts);
  double dist;
  double min_dist_any = std::numeric_limits<double>::infinity();
  Eigen::Vector3d min_pt = Eigen::Vector3d::Zero();
  bool collision = false;
  for (unsigned int i = 0; i < car_pts.size(); ++i) {
    if (precise) {
      dist = grid_map_->getPreciseDistance(car_pts[i]);
    } else {
      dist = grid_map_->getDistance(car_pts[i]);
    }
    if (dist < min_dist_any) {
      min_dist_any = dist;
      min_pt = car_pts[i];
    }
    if (dist < safe_dist) {
      collision = true;
    }
  }
  if (collision) {
    min_dist = min_dist_any;
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("mm_config"), mm_utils::log_clock(), static_cast<int64_t>(1000*(
        1.0)),
        "[CarCollision] state=(%.2f,%.2f,yaw=%.2f) min_dist=%.3f safe_dist=%.3f "
        "car_pt=(%.2f,%.2f,%.2f) precise=%d",
        car_state(0), car_state(1), car_state(2), min_dist_any, safe_dist,
        min_pt.x(), min_pt.y(), min_pt.z(), precise ? 1 : 0);
    return true;
  }
  min_dist = safe_dist;
  return false;
}

bool MMConfig::checkManiObsCollision(Eigen::Vector3d car_state,
                                     Eigen::VectorXd mani_state, bool safe,
                                     double &min_dist, bool record) {
  Eigen::Matrix4d T_q = Eigen::Matrix4d::Identity();
  T_q(0, 0) = cos(car_state(2));
  T_q(0, 1) = -sin(car_state(2));
  T_q(0, 3) = car_state(0);
  T_q(1, 0) = sin(car_state(2));
  T_q(1, 1) = cos(car_state(2));
  T_q(1, 3) = car_state(1);
  double safe_dist = safe ? manipulator_thickness_ + mani_safe_margin_
                          : manipulator_thickness_;
  // safe_dist += map_resolution_ / 2.0;
  geometry_msgs::msg::Point pt;
  Eigen::Vector3d pt_on_link;
  Eigen::Matrix4d T_now = T_q * T_q_0_;
  std::vector<Eigen::Matrix4d> T_joint, T_joint_grad_nouse;
  T_joint.clear();
  getJointTrans(mani_state, T_joint, T_joint_grad_nouse);
  double dist;
  for (int i = 0; i < manipulator_dof_; ++i) {
    T_now = T_now * T_joint[i];
    const auto &local=*arm_indices_[i];
    const auto box=transformedBox(local.minimum,local.maximum,T_now);
    if(box.minimum.z()>ground_safe_dis_+1e-9 &&
       grid_map_->interpolationBoxClear(box.minimum,box.maximum,safe_dist))continue;
    // get ESDF value
    int pts_size = manipulator_link_pts_[i].cols();
    for (int j = 0; j < pts_size; ++j) {
      pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
      pt.x = pt_on_link(0);
      pt.y = pt_on_link(1);
      pt.z = pt_on_link(2);
      if (pt_on_link(2) < ground_safe_dis_) {
        min_dist = pt_on_link(2);
        if(record)sphere_occ_.points.push_back(pt);
        return true;
      }
      dist = grid_map_->getPreciseDistance(pt_on_link);
      // dist = grid_map_->getDistance(pt_on_link);
      if (dist < safe_dist) {
        if(record)sphere_occ_.points.push_back(pt);
        min_dist = dist;
        return true;
      }
    }
  }
  min_dist = safe_dist;
  return false;
}

bool MMConfig::planningEnvironmentCollision(const Eigen::Vector3d &base,const Eigen::VectorXd &q) {
  Eigen::Matrix4d transform;
  CarState2T(base,transform);
  if(body_index_->obstacleHit(0,transform,*grid_map_,mobile_base_check_radius_+map_resolution_,
                              -std::numeric_limits<double>::infinity()))return true;
  transform*=T_q_0_;
  for(int i=0;i<manipulator_dof_;++i) {
    Eigen::Matrix4d joint,unused;
    getAJointTran(i,q[i],joint,unused);
    transform*=joint;
    if(arm_indices_[i]->obstacleHit(0,transform,*grid_map_,manipulator_thickness_,ground_safe_dis_))return true;
  }
  return false;
}

bool MMConfig::planningSelfCollision(const Eigen::VectorXd &q) const {
  if(q.size()!=manipulator_dof_||!q.allFinite())return true;
  std::array<Eigen::Matrix4d,6> poses;
  // The joint model is immutable after loading.
  Eigen::Matrix4d current=T_q_0_;
  const double body_limit=mobile_base_check_radius_+manipulator_thickness_;
  const double arm_limit=2*manipulator_thickness_;
  const auto separated=[](const TransformedBox &box,const BodyIndex &target,double limit) {
    const Eigen::Vector3d delta=(target.minimum-box.maximum).cwiseMax(0.)+
        (box.minimum-target.maximum).cwiseMax(0.);
    return delta.squaredNorm()>(limit+1e-5)*(limit+1e-5);
  };
  for(int a=0;a<manipulator_dof_;++a) {
    Eigen::Matrix4d joint, unused;
    getAJointTran(a,q[a],joint,unused);
    current*=joint;poses[a]=current;
    const auto &source=*arm_indices_[a];
    if(a>0 && !separated(transformedBox(source.minimum,source.maximum,current),*body_index_,body_limit)) {
      for(const auto &local:source.exact_points) {
        const Eigen::Vector3d point=current.topLeftCorner<3,3>()*local+current.topRightCorner<3,1>();
        if(body_index_->anyWithin(0,point,body_limit*body_limit,
             [&](int b){return a>1||body_index_->owners[b]!=1.;}))return true;
      }
    }
    for(int b=0;b<a;++b) {
      if(!hasSelfCollisionPair(a,b))continue;
      const auto &target=*arm_indices_[b];
      const Eigen::Matrix4d relative=poses[b].inverse()*current;
      if(separated(transformedBox(source.minimum,source.maximum,relative),target,arm_limit))continue;
      for(int i=0;i<static_cast<int>(source.exact_points.size());++i) {
        if(!hasSelfCollisionPointPair(a,i,b))continue;
        const Eigen::Vector3d point=relative.topLeftCorner<3,3>()*source.exact_points[i]+relative.topRightCorner<3,1>();
        if(target.anyWithin(0,point,arm_limit*arm_limit,
             [&](int j){return !skipSelfPair(a,i,b,j);}))return true;
      }
    }
  }
  return false;
}

bool MMConfig::checkCarManiCollision(Eigen::VectorXd mani_state, bool safe,
                                     double &min_dist) {
  double safe_dist = safe ? mobile_base_check_radius_ + manipulator_thickness_ +
                                self_safe_margin_
                          : mobile_base_check_radius_ + manipulator_thickness_;
  Eigen::Vector3d pt_on_link;
  std::vector<Eigen::Matrix4d> T_joint, T_joint_grad;
  getJointTrans(mani_state,T_joint,T_joint_grad);
  int pts_size;
  Eigen::Matrix4d T_now = T_q_0_ * T_joint[0];
  for (int i = 1; i < manipulator_dof_; ++i) {
    T_now = T_now * T_joint[i];
    const auto &local=*arm_indices_[i];
    const auto box=transformedBox(local.minimum,local.maximum,T_now);
    const Eigen::Vector3d separation=(body_index_->minimum-box.maximum).cwiseMax(0.)+
        (box.minimum-body_index_->maximum).cwiseMax(0.);
    if(separation.squaredNorm()>(safe_dist+1e-5)*(safe_dist+1e-5))continue;
    pts_size = manipulator_link_pts_[i].cols();
    for (int j = 0; j < pts_size; ++j) {
      pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
      const auto neighbors=baseNeighbors(pt_on_link,safe_dist,i);
      if(!neighbors.empty()) {
        min_dist=(pt_on_link-neighbors.front()).norm();return true;
      }
    }
  }
  min_dist = safe_dist;
  return false;
}

bool MMConfig::checkManiManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist) {
  const double limit=2*manipulator_thickness_+(safe?self_safe_margin_:0.0);
  if (mani_state.size() != manipulator_dof_ || !mani_state.allFinite())
    throw std::invalid_argument("Invalid arm state for self-collision checking");
  std::vector<Eigen::Matrix4d> transforms;
  Eigen::Matrix4d current=Eigen::Matrix4d::Identity();
  for(int i=0; i<manipulator_dof_; ++i) {
    Eigen::Matrix4d joint, gradient;
    getAJointTran(i,mani_state[i],joint,gradient);
    current *= joint;
    transforms.push_back(current);
    for(int j=0; j<i; ++j) {
      if (!hasSelfCollisionPair(i,j)) continue;
      const Eigen::Matrix4d relative = transforms[j].inverse() * current;
      const auto &source=*arm_indices_[i];
      const auto &target=*arm_indices_[j];
      const auto box=transformedBox(source.minimum,source.maximum,relative);
      const Eigen::Vector3d separation=(target.minimum-box.maximum).cwiseMax(0.)+
          (box.minimum-target.maximum).cwiseMax(0.);
      if(separation.squaredNorm()>(limit+1e-5)*(limit+1e-5))continue;
      const Eigen::Matrix3Xd points = (relative * manipulator_link_pts_[i]).topRows<3>();
      for(int a=0; a<points.cols(); ++a) {
        if (!hasSelfCollisionPointPair(i,a,j)) continue;
        for(int b : armNeighbors(j,points.col(a),limit)) {
          if(skipSelfPair(i,a,j,b)) continue;
          const double distance=(points.col(a)-manipulator_link_pts_[j].col(b).head<3>()).norm();
          if(distance<limit) { min_dist=distance; return true; }
        }
      }
    }
  }
  min_dist=limit;return false;
}

bool MMConfig::checkManicollision(Eigen::Vector3d car_state,
                                  Eigen::VectorXd mani_state, bool safe) {
  double min_dist;
  if (checkManiObsCollision(car_state, mani_state, safe, min_dist)) {
    return true;
  }
  if (checkCarManiCollision(mani_state, safe, min_dist)) {
    return true;
  }
  if (checkManiManiCollision(mani_state, safe, min_dist)) {
    return true;
  }
  return false;
}

bool MMConfig::checkcollision(
    Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe,
    int &coll_type /*0: car, 1: mani, 2: car-mani, 3: mani-mani*/) {
  double min_dist;
  if (checkCarObsCollision(car_state, true, safe, min_dist)) {
    coll_type = 0;
    return true;
  }
  if (checkManiObsCollision(car_state, mani_state, safe, min_dist)) {
    coll_type = 1;
    return true;
  }
  if (checkCarManiCollision(mani_state, safe, min_dist)) {
    coll_type = 2;
    return true;
  }
  if (checkManiManiCollision(mani_state, safe, min_dist)) {
    coll_type = 3;
    return true;
  }
  coll_type = -1;
  return false;
}

bool MMConfig::checkcollision(Eigen::Vector3d car_state,
                              Eigen::VectorXd mani_state, bool safe) {
  double min_dist;
  if (checkCarObsCollision(car_state, true, safe, min_dist)) {
    return true;
  }
  if (checkManiObsCollision(car_state, mani_state, safe, min_dist)) {
    return true;
  }
  if (checkCarManiCollision(mani_state, safe, min_dist)) {
    return true;
  }
  if (checkManiManiCollision(mani_state, safe, min_dist)) {
    return true;
  }
  return false;
}


void MMConfig::setGripperPoint(const bool) { /* Fixed camera is included in the last link samples. */ }

void MMConfig::visCarCheckBall(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx,
                               double alpha, const Eigen::Vector3d &state) {
  std::vector<Eigen::Vector3d> car_pts;
  getCarPts(state, car_pts, Eigen::Vector3d::Zero());

  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = "world";
  sphere.header.stamp = node_->now();
  sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.id = idx;

  sphere.pose.orientation.w = 1.0;
  sphere.color.r = 1.0;
  sphere.color.g = 0;
  sphere.color.b = 0;
  sphere.color.r = color_set_[0](0);
  sphere.color.g = color_set_[0](1);
  sphere.color.b = color_set_[0](2);
  sphere.color.a = alpha > 1e-5 ? alpha : 1.0;
  sphere.scale.x = mobile_base_check_radius_ * 2.0;
  sphere.scale.y = mobile_base_check_radius_ * 2.0;
  sphere.scale.z = mobile_base_check_radius_ * 2.0;
  geometry_msgs::msg::Point pt;
  for (unsigned int i = 0; i < car_pts.size(); i++) {
    pt.x = car_pts[i](0);
    pt.y = car_pts[i](1);
    pt.z = car_pts[i](2);
    sphere.points.push_back(pt);
  }
  pub->publish(sphere);
}

void MMConfig::visManiCheckBall(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx,
                                double alpha, const Eigen::Vector3d &car_state,
                                const Eigen::VectorXd &joint_state) {
  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = "world";
  sphere.header.stamp = node_->now();
  sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.id = idx + 1000;

  sphere.pose.orientation.w = 1.0;
  sphere.color.r = 1.0;
  sphere.color.g = 0;
  sphere.color.b = 0;
  sphere.color.a = alpha > 1e-5 ? alpha : 1.0;
  sphere.scale.x = manipulator_thickness_ * 2.0;
  sphere.scale.y = manipulator_thickness_ * 2.0;
  sphere.scale.z = manipulator_thickness_ * 2.0;
  geometry_msgs::msg::Point pt;

  Eigen::Matrix4d T_q;
  T_q << cos(car_state(2)), -sin(car_state(2)), 0.0, car_state(0),
      sin(car_state(2)), cos(car_state(2)), 0.0, car_state(1), 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix4d T_now = T_q * T_q_0_;
  Eigen::Vector3d pt_on_link;

  for (int i = 0; i < manipulator_dof_; ++i) {
    sphere.color.r = color_set_[i + 1](0);
    sphere.color.g = color_set_[i + 1](1);
    sphere.color.b = color_set_[i + 1](2);
    ++sphere.id;
    Eigen::Matrix4d temp, temp_grad;
    getAJointTran(i, joint_state(i), temp, temp_grad);
    T_now = T_now * temp;
    int pts_size = manipulator_link_pts_[i].cols();
    for (int j = 0; j < pts_size; ++j) {
      pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
      pt.x = pt_on_link(0);
      pt.y = pt_on_link(1);
      pt.z = pt_on_link(2);
      sphere.points.push_back(pt);
    }
    pub->publish(sphere);
    sphere.points.clear();
  }
}

visualization_msgs::msg::MarkerArray
MMConfig::getCarMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &state) {
  visualization_msgs::msg::MarkerArray result;
  Eigen::Matrix4d t; CarState2T(state,t);
  result.markers.push_back(getMarker(idx*vis_idx_size_,ns,alpha,t,mesh_resource_mobile_base_));
  return result;
}

visualization_msgs::msg::MarkerArray MMConfig::getManiMarkerArray(
    std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state,
    const Eigen::VectorXd &joint_state, const bool &) {
  visualization_msgs::msg::MarkerArray result;
  Eigen::Matrix4d base; CarState2T(car_state,base);
  Eigen::Matrix4d t=base*T_q_0_;
  for(int i=0;i<manipulator_dof_;++i) {
    Eigen::Matrix4d joint,gradient; getAJointTran(i,joint_state(i),joint,gradient); t*=joint;
    if (i<static_cast<int>(arm_mesh_resources_.size()))
      result.markers.push_back(getMarker(idx*vis_idx_size_+i+1,ns,alpha,t,arm_mesh_resources_[i]));
  }
  return result;
}

void MMConfig::visMesh(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, int id, std::string ns,
                       double alpha, Eigen::Vector3d color_rgb,
                       const Eigen::Matrix4d &T, const std::string &mesh_file) {
  Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
  Eigen::Quaterniond quad;
  quad = rotation_matrix;
  visualization_msgs::msg::Marker meshMarker;
  meshMarker.header.frame_id = "world";
  meshMarker.header.stamp = node_->now();
  meshMarker.ns = ns;
  meshMarker.id = id;
  meshMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  meshMarker.action = visualization_msgs::msg::Marker::ADD;
  meshMarker.mesh_use_embedded_materials = true;
  meshMarker.pose.position.x = T(0, 3);
  meshMarker.pose.position.y = T(1, 3);
  meshMarker.pose.position.z = T(2, 3);
  meshMarker.pose.orientation.w = quad.w();
  meshMarker.pose.orientation.x = quad.x();
  meshMarker.pose.orientation.y = quad.y();
  meshMarker.pose.orientation.z = quad.z();
  meshMarker.scale.x = 1.0;
  meshMarker.scale.y = 1.0;
  meshMarker.scale.z = 1.0;
  if (id == 2 || id == 102) {
    meshMarker.scale.x = 0.001;
    meshMarker.scale.y = 0.001;
    meshMarker.scale.z = 0.001;
  }
  // meshMarker.color.r = color_rgb(0);
  // meshMarker.color.g = color_rgb(1);
  // meshMarker.color.r = color_rgb(2);
  if (alpha >= 0.0)
    meshMarker.color.a = alpha;
  meshMarker.mesh_resource = mesh_file;
  pub->publish(meshMarker);
}

visualization_msgs::msg::Marker MMConfig::getMarker(int id, std::string ns,
                                               double alpha,
                                               const Eigen::Matrix4d &T,
                                               const std::string &mesh_file) {
  Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
  Eigen::Quaterniond quad;
  quad = rotation_matrix;
  visualization_msgs::msg::Marker meshMarker;
  meshMarker.header.frame_id = "world";
  meshMarker.header.stamp = node_->now();
  meshMarker.ns = ns;
  meshMarker.id = id;
  meshMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  meshMarker.action = visualization_msgs::msg::Marker::ADD;
  meshMarker.mesh_use_embedded_materials = true;
  meshMarker.pose.position.x = T(0, 3);
  meshMarker.pose.position.y = T(1, 3);
  meshMarker.pose.position.z = T(2, 3);
  meshMarker.pose.orientation.w = quad.w();
  meshMarker.pose.orientation.x = quad.x();
  meshMarker.pose.orientation.y = quad.y();
  meshMarker.pose.orientation.z = quad.z();
  meshMarker.scale.x = 1.0;
  meshMarker.scale.y = 1.0;
  meshMarker.scale.z = 1.0;
  if (alpha >= 0.0)
    meshMarker.color.a = alpha;
  meshMarker.mesh_resource = mesh_file;
  return meshMarker;
}

} // namespace remani_planner
