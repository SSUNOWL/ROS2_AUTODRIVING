cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(OsqpEigen REQUIRED) # OSQP Wrapper
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include 디렉토리 설정
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIRS}
)

# ---------------------------------------------------------
# 2. 라이브러리 생성 (물리 엔진 & Solver)
# ---------------------------------------------------------

# (A) Dynamic Model 라이브러리
add_library(mpc_dynamics src/dynamic_model.cpp)
target_include_directories(mpc_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# (B) MPC Solver 라이브러리 [이 부분이 빠져있어서 에러가 났었습니다!]
add_library(mpc_solver_lib src/mpc_solver.cpp)
target_include_directories(mpc_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIRS}
)
# Solver는 Dynamic Model과 OSQP를 사용하므로 링크 연결
target_link_libraries(mpc_solver_lib 
  mpc_dynamics 
  OsqpEigen::OsqpEigen
)

# ---------------------------------------------------------
# 3. 실행 파일(Node) 생성
# ---------------------------------------------------------
add_executable(mpc_node src/mpc_node.cpp)

# 의존성 연결
ament_target_dependencies(mpc_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  Eigen3
  ackermann_msgs
  sensor_msgs
  visualization_msgs
)

# 우리가 만든 라이브러리(Solver + Dynamics) 링크
target_link_libraries(mpc_node 
  mpc_solver_lib
  mpc_dynamics
  OsqpEigen::OsqpEigen
)

# ---------------------------------------------------------
# 4. 설치 설정 (Install)
# ---------------------------------------------------------
install(TARGETS
  mpc_dynamics
  mpc_solver_lib
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()