cmake_minimum_required(VERSION 3.8)
project(offboard_demo)

# ===================================================
# COMPILER OPTIONS
# ===================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
# FIND DEPENDENCIES
# ===================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# ===================================================
# BUILD EXECUTABLE NODES
# ===================================================
# Main offboard control node
add_executable(offboard_demo 
  src/offboard_demo.cpp
)
ament_target_dependencies(offboard_demo rclcpp px4_msgs)

# Local position control node
add_executable(offboard_demo_local_position 
  src/offboard_demo_local_position.cpp
)
ament_target_dependencies(offboard_demo_local_position rclcpp px4_msgs)

# Odometry subscriber node
add_executable(offboard_demo_odometry 
  src/offboard_demo_odometry.cpp
)
ament_target_dependencies(offboard_demo_odometry rclcpp px4_msgs)

# Auto-land control node
add_executable(offboard_demo_autoland 
  src/offboard_demo_autoland.cpp
)
ament_target_dependencies(offboard_demo_autoland rclcpp px4_msgs)

# ===================================================
# INSTALL TARGETS
# ===================================================
install(TARGETS
  offboard_demo
  offboard_demo_local_position
  offboard_demo_odometry
  offboard_demo_autoland
  DESTINATION lib/${PROJECT_NAME}
)

# ===================================================
# TESTING CONFIGURATION
# ===================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()