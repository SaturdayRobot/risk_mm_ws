#include "mm_bringup/sim_control.hpp"
#include <iostream>
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {auto control=risk_mm::make_sim_control();rclcpp::spin(control.node);}
  catch(const std::exception &error){std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();
}
