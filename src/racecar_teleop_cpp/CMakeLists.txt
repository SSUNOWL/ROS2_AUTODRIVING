cmake_minimum_required(VERSION 3.8)
project(racecar_teleop_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# 실행 파일 생성 설정 (여기부터 추가/수정)
add_executable(teleop_converter src/teleop_converter.cpp)
add_executable(direct_teleop src/direct_teleop.cpp)
ament_target_dependencies(direct_teleop rclcpp ackermann_msgs)
ament_target_dependencies(teleop_converter rclcpp geometry_msgs ackermann_msgs)

add_executable(opp_teleop src/opp_teleop.cpp)
ament_target_dependencies(opp_teleop rclcpp ackermann_msgs)
install(TARGETS
  direct_teleop
  opp_teleop
  teleop_converter
  DESTINATION lib/${PROJECT_NAME})
ament_package()