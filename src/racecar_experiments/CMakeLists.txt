cmake_minimum_required(VERSION 3.8)
project(racecar_experiments)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(collision_monitor src/collision_monitor.cpp)
ament_target_dependencies(collision_monitor rclcpp sensor_msgs std_msgs)

add_executable(path_timer src/path_timer.cpp)
ament_target_dependencies(path_timer rclcpp nav_msgs std_msgs)

add_executable(path_saver src/path_saver.cpp)
ament_target_dependencies(path_saver rclcpp nav_msgs)

add_executable(static_path_publisher src/static_path_publisher.cpp)
ament_target_dependencies(static_path_publisher rclcpp nav_msgs geometry_msgs)


add_executable(run_logger src/run_logger.cpp)
ament_target_dependencies(run_logger rclcpp nav_msgs geometry_msgs)


install(TARGETS
  collision_monitor
  path_timer
  path_saver
  static_path_publisher
  run_logger
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
