#pragma once
#include "remani_adapter/planner.hpp"
#include "risk_mm_runtime/world_scene.hpp"
#include <memory>

namespace risk_mm {
struct ValidationLimits {
  double sample_s{.02}, arm_velocity{.35}, arm_acceleration{.7};
  double base_velocity{1.0}, base_yaw_rate{1.2};
  double wheel_velocity{10.0}, wheel_acceleration{5.0}, steer_rate{1.5707963267948966};
  double collision_padding{.01};
  double wheel_radius{.1};
  std::array<double,8> wheel_positions{};
};
struct ValidationReport {
  size_t samples{0};
  double wall_s{0}, max_arm_velocity{0},max_arm_acceleration{0};
  double max_wheel_velocity{0},max_wheel_acceleration{0},max_steer_rate{0};
  std::array<double,4> end_steer{},end_wheel_position{};
};

class MeshScene {
  class Implementation;
  std::unique_ptr<Implementation> impl_;
 public:
  MeshScene(const WorldScene &,const std::string &urdf_file,const std::string &srdf_file,double padding);
  ~MeshScene();
  bool valid(const std::array<double,9> &,const std::array<double,4> &steer,
             const std::array<double,4> &wheels,std::string &reason,
             double base_z=0.,double roll=0.,double pitch=0.);
};

class SampledValidator final : public Validator {
  std::shared_ptr<MeshScene> scene_;
  ValidationLimits limits_;
  mutable ValidationReport report_;
  std::string model_signature_;
  unsigned long map_revision_;
 public:
  SampledValidator(std::shared_ptr<MeshScene>,ValidationLimits,std::string model_signature,unsigned long map_revision);
  bool validate(const Plan &,std::string &reason) const override;
  const ValidationReport &report() const {return report_;}
};
}
