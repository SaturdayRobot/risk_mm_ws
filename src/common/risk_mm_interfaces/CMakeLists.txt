cmake_minimum_required(VERSION 3.16)
project(risk_mm_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanCoupled.srv" "srv/ExecutePlan.srv" "srv/StartPatrol.srv" "msg/CoupledStatus.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
