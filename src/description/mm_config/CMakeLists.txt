cmake_minimum_required(VERSION 3.8)
project(mm_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mm_utils REQUIRED)
find_package(plan_env REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)

# ---------------- 库 ----------------
add_library(mm_config
  src/mm_config.cpp)
target_include_directories(mm_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mm_config rclcpp mm_utils plan_env visualization_msgs)
if(PCL_LIBRARIES)
  target_link_libraries(mm_config ${PCL_LIBRARIES})
endif()
install(TARGETS mm_config EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---------------- 可执行 ----------------
# (无可执行)



install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(DIRECTORY config launch scripts rviz DESTINATION share/${PROJECT_NAME}
        OPTIONAL)

ament_export_dependencies(rclcpp mm_utils plan_env visualization_msgs)
add_executable(model_check test/model_check.cpp)
ament_target_dependencies(model_check rclcpp mm_utils plan_env visualization_msgs)
target_link_libraries(model_check mm_config)
install(TARGETS model_check DESTINATION lib/${PROJECT_NAME})
ament_package()
