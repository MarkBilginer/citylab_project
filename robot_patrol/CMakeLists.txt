cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(include)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/patrol.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
 rclcpp
 std_msgs  sensor_msgs
 geometry_msgs
 custom_interfaces
)

add_executable(direction_service_server_node src/direction_service.cpp)
ament_target_dependencies(direction_service_server_node
  rclcpp
  std_msgs
  sensor_msgs
  custom_interfaces
)

add_executable(test_service_client_node src/test_service.cpp)
ament_target_dependencies(test_service_client_node
  rclcpp
  #std_msgs
  sensor_msgs
  custom_interfaces
)


add_executable(robot_patrol_with_service_client_node src/patrol_with_service.cpp)
ament_target_dependencies(robot_patrol_with_service_client_node
  rclcpp
  geometry_msgs
  sensor_msgs
  custom_interfaces
)

add_executable(go_to_pose_action src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action rclcpp geometry_msgs nav_msgs rclcpp_action)

# Install the executable and necessary directories
install(TARGETS
  robot_patrol_node
  direction_service_server_node
  test_service_client_node
  robot_patrol_with_service_client_node
  go_to_pose_action
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
