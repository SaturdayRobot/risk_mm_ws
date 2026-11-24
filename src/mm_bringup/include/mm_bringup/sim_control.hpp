#pragma once
#include <rclcpp/rclcpp.hpp>
#include <risk_mm_runtime/execution.hpp>

namespace risk_mm {
struct SimControlHandle {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<ActuatorPort> actuator;
};
SimControlHandle make_sim_control(const rclcpp::NodeOptions &options=rclcpp::NodeOptions(),
                                  const std::string &model_signature="");
}
