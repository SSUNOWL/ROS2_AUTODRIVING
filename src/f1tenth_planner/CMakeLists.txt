cmake_minimum_required(VERSION 3.8)
project(f1tenth_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED) # 마커 사용 시 필요

# 시스템 라이브러리 찾기 (OpenCV, Yaml-cpp) - offline_planner용
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# =========================================
# 2. 실행 파일 생성 (Build Targets)
# =========================================

# (1) Offline Planner
add_executable(offline_planner src/offline_planner.cpp)
target_include_directories(offline_planner PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
ament_target_dependencies(offline_planner
  rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs
)
target_link_libraries(offline_planner
  ${OpenCV_LIBS}
  yaml-cpp
)

# (2) FGM Node (여기가 빠져서 에러가 났던 것)
add_executable(fgm_node src/fgm_node.cpp)
ament_target_dependencies(fgm_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# (3) Pure Pursuit Node (이것도 필요하실 것 같아 추가함)
add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node
  rclcpp
  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# =========================================
# 3. 설치 설정 (Install)
# =========================================
install(TARGETS
  offline_planner
  fgm_node
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 폴더 등이 있다면 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()