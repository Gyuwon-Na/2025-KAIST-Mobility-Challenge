cmake_minimum_required(VERSION 3.8)
project(bisa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ==========================================
# Custom Messages
# ==========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LapInfo.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

set(dependencies
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ==========================================
# C++ Library: MPC Controller
# ==========================================
add_library(mpc_controller_lib SHARED
  src/mpc_controller.cpp
)
ament_target_dependencies(mpc_controller_lib ${dependencies})

# ==========================================
# C++ Executables
# ==========================================
add_executable(local_path_pub src/local_path_pub.cpp)
ament_target_dependencies(local_path_pub ${dependencies})
rosidl_target_interfaces(local_path_pub ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(mpc_path_tracker src/mpc_path_tracker.cpp)
ament_target_dependencies(mpc_path_tracker ${dependencies})
target_link_libraries(mpc_path_tracker mpc_controller_lib)

add_executable(visualize_hvs src/visualize_hvs.cpp)
ament_target_dependencies(visualize_hvs rclcpp geometry_msgs visualization_msgs)

add_executable(frenet_planner src/frenet_planner.cpp)
ament_target_dependencies(frenet_planner rclcpp geometry_msgs tf2 tf2_geometry_msgs nav_msgs visualization_msgs)

install(TARGETS
  mpc_controller_lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  local_path_pub
  mpc_path_tracker
  visualize_hvs
  frenet_planner
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# Python
# ==========================================

install(PROGRAMS
  scripts/hdmap_visualizer.py
  scripts/global_path_pub.py
  scripts/dijkstra_planner.py
  scripts/mgeo_class_defs.py
  scripts/mpc_tuner_gui.py
  scripts/lane_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch rviz hdmap_data config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
