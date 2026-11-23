cmake_minimum_required(VERSION 3.8)
project(mm_model)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
