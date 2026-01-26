cmake_minimum_required(VERSION 3.5)
project(bisa)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LapInfo.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# mpc_controller_cpp (라이브러리)
add_library(mpc_controller_cpp SHARED
  src/mpc_controller_cpp.cpp
)
ament_target_dependencies(mpc_controller_cpp
  rclcpp geometry_msgs nav_msgs
)
target_include_directories(mpc_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# mpc_path_tracker_cpp
add_executable(mpc_path_tracker_cpp
  src/mpc_path_tracker_cpp.cpp
)
ament_target_dependencies(mpc_path_tracker_cpp
  rclcpp std_msgs geometry_msgs nav_msgs
)
target_link_libraries(mpc_path_tracker_cpp mpc_controller_cpp)
target_include_directories(mpc_path_tracker_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# local_path_pub_cpp
add_executable(local_path_pub_cpp
  src/local_path_pub_cpp.cpp
)
ament_target_dependencies(local_path_pub_cpp
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs
)
target_include_directories(local_path_pub_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 메시지 의존성
if(TARGET ${PROJECT_NAME})
  rosidl_target_interfaces(local_path_pub_cpp
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

# ================================================================
# collision_avoidance_node_cpp (v34.0)
# ================================================================
add_executable(collision_avoidance_node_cpp
  src/collision_avoidance_node.cpp
)
ament_target_dependencies(collision_avoidance_node_cpp
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)
target_include_directories(collision_avoidance_node_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 메시지 의존성 (LapInfo.msg)
rosidl_target_interfaces(collision_avoidance_node_cpp
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# ================================================================

# ================================================================

# Python 스크립트 설치
install(PROGRAMS
  scripts/hdmap_visualizer.py
  scripts/global_path_pub.py
  scripts/global_path_pub_multi.py 
  scripts/dijkstra_planner.py
  scripts/mgeo_class_defs.py
  scripts/mpc_tuner_gui.py
  scripts/collision_avoidance_node.py
  scripts/hdmap_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++ 실행 파일 설치
install(TARGETS
  local_path_pub_cpp
  mpc_path_tracker_cpp
  collision_avoidance_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# C++ 라이브러리 설치 (중요!)
install(TARGETS
  mpc_controller_cpp
  EXPORT mpc_controller_cpp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include
)

# Launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# RViz 설정 설치
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# HDMap 데이터 설치
install(DIRECTORY hdmap_data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()