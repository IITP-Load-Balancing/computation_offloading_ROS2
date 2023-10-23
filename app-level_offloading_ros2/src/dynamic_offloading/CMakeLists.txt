cmake_minimum_required(VERSION 3.5)
project(dynamic_offloading)

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
#### added
find_package(header_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sensor src/sensor.cpp)
ament_target_dependencies(sensor rclcpp std_msgs header_interfaces)

add_executable(computinglocal src/computinglocal.cpp)
ament_target_dependencies(computinglocal rclcpp std_msgs header_interfaces)

add_executable(computingserver src/computingserver.cpp)
ament_target_dependencies(computingserver rclcpp parameter std_msgs header_interfaces)

add_executable(arbitrator src/arbitrator.cpp)
ament_target_dependencies(arbitrator rclcpp std_msgs header_interfaces)

install(TARGETS
  sensor
  computinglocal
  computingserver
  arbitrator
  DESTINATION lib/${PROJECT_NAME})

#### added
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
