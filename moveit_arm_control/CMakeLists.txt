cmake_minimum_required(VERSION 3.5)
project(moveit_arm_control)

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

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(shared_message REQUIRED)

add_executable(ahpc_planning_node src/ahpc_planning_node.cpp)
ament_target_dependencies(
  ahpc_planning_node
  "moveit_core"
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "control_msgs"
  "shared_message"
  )
target_include_directories(ahpc_planning_node PUBLIC
                          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/moveit_arm_control>
                          $<INSTALL_INTERFACE:include/moveit_arm_control>)



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

ament_package()
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS ahpc_planning_node
  DESTINATION lib/${PROJECT_NAME})

