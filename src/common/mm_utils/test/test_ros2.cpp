#include "mm_utils/ros2_utils.hpp"
#include <iostream>
int main(int argc,char**argv) {
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("native_ros2_check");
 node->declare_parameter<int>("mm.mobile_base_dof",2);
 int calls=0;
 auto timer=rclcpp::create_timer(node,node->get_clock(),rclcpp::Duration::from_seconds(.001),
   std::function<void()>([&calls](){++calls;}));
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
 const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(1);
 while(calls<5 && std::chrono::steady_clock::now()<end)executor.spin_some();
 timer->cancel();int stopped=calls;for(int i=0;i<10;++i)executor.spin_some();
 const bool pass=calls>=5 && calls==stopped && node->get_parameter("mm.mobile_base_dof").as_int()==2;
 std::cout<<"native_timer_callbacks="<<calls<<" native_parameter="<<node->get_parameter("mm.mobile_base_dof").as_int()<<std::endl;
 rclcpp::shutdown();return pass?0:1;
}
