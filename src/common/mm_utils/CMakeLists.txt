cmake_minimum_required(VERSION 3.8)
project(mm_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------- 库 ----------------
add_library(mm_utils INTERFACE)
target_include_directories(mm_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mm_utils INTERFACE rclcpp ament_index_cpp nav_msgs geometry_msgs)

# ---------------- 可执行 ----------------
# (无可执行)



install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
install(DIRECTORY config launch scripts rviz DESTINATION share/${PROJECT_NAME}
        OPTIONAL)

ament_export_dependencies(rclcpp ament_index_cpp nav_msgs geometry_msgs)
add_executable(native_ros2_check test/test_ros2.cpp)
target_link_libraries(native_ros2_check mm_utils)
install(TARGETS native_ros2_check DESTINATION lib/${PROJECT_NAME})
ament_package()
