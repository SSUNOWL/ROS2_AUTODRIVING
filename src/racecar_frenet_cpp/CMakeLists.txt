cmake_minimum_required(VERSION 3.8)
project(racecar_frenet_cpp)

# C++17 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ROS2 의존성 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 실행 파일 정의
add_executable(frenet_local_planner src/frenet_local_planner.cpp)

# 실행 파일 의존성
ament_target_dependencies(frenet_local_planner
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  Eigen3::Eigen
)

# 실행 파일 설치
install(TARGETS
  frenet_local_planner
  DESTINATION lib/${PROJECT_NAME}
)

# config 폴더와 launch 폴더 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 패키지 설치
ament_package()
