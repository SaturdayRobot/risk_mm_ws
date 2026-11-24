#pragma once
#include "risk_mm_runtime/execution.hpp"
#include <rclcpp/rclcpp.hpp>

namespace risk_mm {
class CoupledRuntime final : public rclcpp::Node {
  class Implementation;
  std::unique_ptr<Implementation> impl_;
 public:
  explicit CoupledRuntime(const rclcpp::NodeOptions &);
  ~CoupledRuntime();
  std::string modelSignature() const;
  void initialize(std::shared_ptr<ActuatorPort>);
  rclcpp::Node::SharedPtr backendNode() const;
  void stop();
};
}
