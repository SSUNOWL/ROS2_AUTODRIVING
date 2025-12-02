cmake_minimum_required(VERSION 3.8)
project(planner_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(planner_mux_node src/planner_mux_node.cpp)

ament_target_dependencies(planner_mux_node
  rclcpp
  nav_msgs
  sensor_msgs
)

install(TARGETS
  planner_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
