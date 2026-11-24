#include "mm_bringup/sim_control.hpp"
#include <risk_mm_runtime/runtime.hpp>
#include <iostream>

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    rclcpp::NodeOptions options;options.automatically_declare_parameters_from_overrides(true);
    auto runtime=std::make_shared<risk_mm::CoupledRuntime>(options);
    auto control=risk_mm::make_sim_control(options,runtime->modelSignature());
    runtime->initialize(control.actuator);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
    executor.add_node(control.node);executor.add_node(runtime);executor.add_node(runtime->backendNode());
    executor.spin();runtime->stop();
  } catch(const std::exception &error) {
    std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
