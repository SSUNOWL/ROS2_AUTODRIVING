cmake_minimum_required(VERSION 3.8)
project(f1tenth_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# 2. 실행 파일(Target) 먼저 정의하기 (이 부분이 위로 와야 함!)
add_executable(offline_planner src/offline_planner.cpp)
ament_target_dependencies(offline_planner rclcpp nav_msgs geometry_msgs)


add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node 
  rclcpp ackermann_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs visualization_msgs
)

# 4. Include 디렉토리 설정 (Target 정의 후 실행)
target_include_directories(offline_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# 5. 외부 라이브러리 링크 (Target 정의 후 실행)
target_link_libraries(offline_planner
  ${OpenCV_LIBS}
  yaml-cpp
)

# 6. 설치 설정
install(TARGETS 
  offline_planner
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()