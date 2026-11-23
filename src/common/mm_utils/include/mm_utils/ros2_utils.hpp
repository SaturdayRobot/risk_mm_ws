#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <chrono>
#include <thread>
#include <functional>
#include <cmath>
#include <cstdlib>

namespace mm_utils {
// Small value helpers only: nodes, publishers, subscriptions and timers use
// their native ROS 2 types and are owned by their corresponding components.
inline double yaw(const geometry_msgs::msg::Quaternion &q) {
 return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
inline rclcpp::Time time_from_seconds(double seconds) {
 return rclcpp::Time(static_cast<int64_t>(seconds*1e9),RCL_ROS_TIME);
}
inline rclcpp::Clock &log_clock() {
 static rclcpp::Clock clock(RCL_STEADY_TIME);
 return clock;
}
}
