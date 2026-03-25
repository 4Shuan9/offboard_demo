cmake_minimum_required(VERSION 3.8)
project(offboard_demo)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Build executables
add_executable(offboard_demo 
  src/offboard_demo.cpp
)
ament_target_dependencies(offboard_demo rclcpp px4_msgs)

add_executable(offboard_demo_local_position 
  src/offboard_demo_local_position.cpp
)
ament_target_dependencies(offboard_demo_local_position rclcpp px4_msgs)

add_executable(offboard_demo_odometry 
  src/offboard_demo_odometry.cpp
)
ament_target_dependencies(offboard_demo_odometry rclcpp px4_msgs)

add_executable(offboard_demo_auto_land 
  src/offboard_demo_auto_land.cpp
)
ament_target_dependencies(offboard_demo_auto_land rclcpp px4_msgs)

# Install executables
install(TARGETS
  offboard_demo
  offboard_demo_local_position
  offboard_demo_odometry
  offboard_demo_auto_land
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_testing_dependencies()
endif()

ament_package()