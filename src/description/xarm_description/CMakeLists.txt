cmake_minimum_required(VERSION 3.8)
project(xarm_description)
find_package(ament_cmake REQUIRED)
# Trimmed resource package: only assets referenced by the current whole robot.
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
