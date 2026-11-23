cmake_minimum_required(VERSION 3.8)
project(plan_env)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mm_utils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# ---------------- 库 ----------------
add_library(plan_env
  src/grid_map.cpp
  src/raycast.cpp)
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(plan_env rclcpp mm_utils cv_bridge message_filters pcl_conversions geometry_msgs nav_msgs sensor_msgs visualization_msgs)
if(PCL_LIBRARIES)
  target_link_libraries(plan_env ${PCL_LIBRARIES})
endif()
install(TARGETS plan_env EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---------------- 可执行 ----------------
# (无可执行)



install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(DIRECTORY config launch scripts rviz DESTINATION share/${PROJECT_NAME}
        OPTIONAL)

ament_export_dependencies(rclcpp mm_utils cv_bridge message_filters pcl_conversions geometry_msgs nav_msgs sensor_msgs visualization_msgs)
ament_package()
