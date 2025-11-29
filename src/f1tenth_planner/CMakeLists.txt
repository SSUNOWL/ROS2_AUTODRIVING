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

# 시스템 라이브러리 찾기 (OpenCV, Yaml-cpp)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# 2. 실행 파일 생성
add_executable(offline_planner src/offline_planner.cpp)

# 3. Include 디렉토리 설정 (OpenCV 등)
target_include_directories(offline_planner PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR} # 시스템 환경에 따라 다를 수 있으나 보통 자동 처리됨
)

# 4. ROS 2 패키지 의존성 연결 (여기서 rclcpp 헤더 경로가 연결됨)
ament_target_dependencies(offline_planner
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# 5. 시스템 라이브러리 링크 연결
target_link_libraries(offline_planner
  ${OpenCV_LIBS}
  yaml-cpp
)

# 6. 설치 설정
install(TARGETS
  offline_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()