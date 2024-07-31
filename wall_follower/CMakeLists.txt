cmake_minimum_required(VERSION 3.5)
project(wall_follower)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(wall_follower_node src/wall_following.cpp)
ament_target_dependencies(wall_follower_node custom_interface rclcpp std_msgs geometry_msgs sensor_msgs rclcpp_action)

add_executable(wall_finder_node src/wall_finder.cpp)
ament_target_dependencies(wall_finder_node custom_interface rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(odom_recorder_server src/odom_recorder.cpp)
ament_target_dependencies(odom_recorder_server nav_msgs custom_interface rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs)


install(TARGETS
  wall_finder_node
  odom_recorder_server
  wall_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
