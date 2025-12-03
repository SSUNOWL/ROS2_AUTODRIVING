cmake_minimum_required(VERSION 3.8)
project(racecar_experiments)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(planner_mux_msgs REQUIRED)

# ---- static_path_publisher ----
add_executable(static_path_publisher src/static_path_publisher.cpp)
ament_target_dependencies(static_path_publisher
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
)

# ---- collision_monitor ----
add_executable(collision_monitor src/collision_monitor.cpp)
ament_target_dependencies(collision_monitor
  rclcpp
  sensor_msgs
  ackermann_msgs
  std_msgs
)

# ---- path_timer ----
add_executable(path_timer src/path_timer.cpp)
ament_target_dependencies(path_timer
  rclcpp
  nav_msgs
  std_msgs
)

# ---- run_logger (MUX 튜닝용 로그 노드) ----
add_executable(run_logger src/run_logger.cpp)
ament_target_dependencies(run_logger
  rclcpp
  nav_msgs
  ackermann_msgs
  planner_mux_msgs
  std_msgs
)

# ---- install ----
install(TARGETS
  static_path_publisher
  collision_monitor
  path_timer
  run_logger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
