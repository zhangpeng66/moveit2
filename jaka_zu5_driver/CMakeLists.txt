cmake_minimum_required(VERSION 3.5)
project(jaka_zu5_driver)

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
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shared_message REQUIRED)
find_package(backward_ros REQUIRED)
find_package(OpenCV 4.5.5 EXACT QUIET)
if(NOT OpenCV_FOUND)
  # 找不到再指定目录, 防止不在算法规定的位置安装
  set(OpenCV_DIR "/usr/local/opencv/opencv-4.5.5/lib/cmake/opencv4")
  set(OpenCV_INCLUDE_DIRS "/usr/local/opencv/opencv-4.5.5/include")
  find_package(OpenCV 4.5.5 EXACT REQUIRED)
endif()
find_package(
  Boost
  COMPONENTS regex system thread filesystem
  REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# get cpu arch info, using cooresponding library
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "aarch64 cpu, link aarch64 libs")
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/aarch64)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  message(STATUS "x86_64 cpu, link x86_64 libs")

endif()
add_library(${PROJECT_NAME} SHARED src/jaka_zu5_driver.cpp)
include_directories(${OpenCV_INCLUDE_DIRS} 
                     include 
                     include/soft_inserter_interface
                     #include/console 
                     "/usr/include/eigen3")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  ament_target_dependencies(
    ${PROJECT_NAME} PUBLIC
    "rclcpp"
    "rclcpp_components"
    "rclcpp_action"
    "std_msgs"
    "std_srvs"
    "sensor_msgs"
    "control_msgs"
    "geometry_msgs"
    "shared_message"
    "backward_ros")

endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
target_link_libraries(${PROJECT_NAME} PUBLIC 
  jakaAPI
  soft_inserter_interface
  ${OpenCV_LIBS}
  z
  ${Boost_LIBRARIES})
endif()


#exe node          
add_executable(jaka_zu5_driver_node src/jaka_zu5_driver_node.cpp)
target_link_libraries(jaka_zu5_driver_node PUBLIC
  ${PROJECT_NAME}
)

set_target_properties(jaka_zu5_driver_node  ${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)

target_include_directories(jaka_zu5_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  # get cpu arch info, using cooresponding library
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "aarch64 cpu, install aarch64 libs")
  install(DIRECTORY lib/linux/aarch64/ DESTINATION lib/${PROJECT_NAME})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  message(STATUS "x86_64 cpu, install x86_64 libs")
  install(DIRECTORY lib/linux/x86_64/ DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS jaka_zu5_driver_node ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files.
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# find_package(ament_cmake_python REQUIRED)
# ament_python_install_package(jaka_launch_utils)
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
