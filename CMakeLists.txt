cmake_minimum_required(VERSION 3.8)
project(tf2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)  # Example dependency
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(listener_cpp src/listener_cpp.cpp)
ament_target_dependencies(listener_cpp rclcpp geometry_msgs tf2 tf2_ros)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/listener_python.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  listener_cpp
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
