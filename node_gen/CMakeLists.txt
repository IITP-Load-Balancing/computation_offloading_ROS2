cmake_minimum_required(VERSION 3.5)
project(node_gen)

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

add_executable(dynamic_node src/dynamic_node.cpp)
ament_target_dependencies(dynamic_node rclcpp std_msgs)

install(TARGETS
  dynamic_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(dynamic_pub src/dynamic_pub.cpp)
ament_target_dependencies(dynamic_pub rclcpp std_msgs)

install(TARGETS
  dynamic_pub
  DESTINATION lib/${PROJECT_NAME})

# New executable files
add_executable(variable_payload_pub src/variable_payload_pub.cpp)
ament_target_dependencies(variable_payload_pub rclcpp std_msgs)

install(TARGETS
  variable_payload_pub
  DESTINATION lib/${PROJECT_NAME})

add_executable(variable_payload_sub src/variable_payload_sub.cpp)
ament_target_dependencies(variable_payload_sub rclcpp std_msgs)

install(TARGETS
  variable_payload_sub
  DESTINATION lib/${PROJECT_NAME})

# New executable file for payload_pub
add_executable(payload_pub src/payload_pub.cpp)
ament_target_dependencies(payload_pub rclcpp std_msgs)

install(TARGETS
  payload_pub
  DESTINATION lib/${PROJECT_NAME})

# New executable file for static_payload_pub
add_executable(static_payload_pub src/static_payload_pub.cpp)
ament_target_dependencies(static_payload_pub rclcpp std_msgs)

install(TARGETS
  static_payload_pub
  DESTINATION lib/${PROJECT_NAME})

# New executable file for simple_publisher
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

install(TARGETS
  simple_publisher
  DESTINATION lib/${PROJECT_NAME})

# New executable file for nsub_npub
add_executable(nsub_npub src/nsub_npub.cpp)
ament_target_dependencies(nsub_npub rclcpp std_msgs)

install(TARGETS
  nsub_npub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

