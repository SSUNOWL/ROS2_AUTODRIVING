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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()


add_executable(path_saver src/path_saver.cpp)
ament_target_dependencies(path_saver rclcpp nav_msgs)
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

add_executable(oppteleop_recorder
    src/oppteleop_recorder.cpp
)

add_executable(opponent_replay_node
    src/opponent_replay_node.cpp
)

ament_target_dependencies(opponent_replay_node ackermann_msgs rclcpp nav_msgs )


ament_target_dependencies(oppteleop_recorder ackermann_msgs rclcpp nav_msgs )


add_executable(opponent_pure_pursuit_node
    src/opponent_pure_pursuit_node.cpp
)

ament_target_dependencies(opponent_pure_pursuit_node   
  rclcpp
  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
 )

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
  oppteleop_recorder
  opponent_replay_node
  opponent_pure_pursuit_node

  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
