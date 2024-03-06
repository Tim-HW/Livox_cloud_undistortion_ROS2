cmake_minimum_required(VERSION 3.5)
project(undistorded-livox-ros2)

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

# find dependencies for ROS compilation
find_package(ament_cmake REQUIRED)
# Find dependencies for C++ and ROS
find_package(rclcpp REQUIRED)
# Find dependencies for ROS messages format
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# external from ROS
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)



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

# Add executable the generate | name must be the same as the launch file
add_executable(undistorded_node src/livox_repub.cpp)
# Link dependencies to the executable | name must be the same as the launch file
ament_target_dependencies(undistorded_node rclcpp std_msgs sensor_msgs)

# install the executable
install(TARGETS
undistorded_node
  DESTINATION lib/${PROJECT_NAME})
# install the launch executable
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)



ament_package()
