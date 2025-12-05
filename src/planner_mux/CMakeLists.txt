cmake_minimum_required(VERSION 3.8)
project(planner_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1) 노드 타겟
add_executable(planner_mux_node src/planner_mux_node.cpp)

ament_target_dependencies(planner_mux_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
)

# 2) 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MuxStatus.msg"
  DEPENDENCIES std_msgs
)

# 3) 새 스타일 typesupport 연결
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(planner_mux_node
  ${cpp_typesupport_target}
)

# 4) install & export
install(TARGETS
  planner_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
