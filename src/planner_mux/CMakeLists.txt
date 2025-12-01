cmake_minimum_required(VERSION 3.8)
project(planner_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# --- [여기부터 추가/수정] ---

# 실행 파일(Executable) 생성 설정
add_executable(planner_mux_node src/planner_mux_node.cpp)

# 의존성 연결
ament_target_dependencies(planner_mux_node rclcpp nav_msgs)

# 설치(Install) 경로 설정
install(TARGETS
  planner_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- [여기까지] ---

ament_package()