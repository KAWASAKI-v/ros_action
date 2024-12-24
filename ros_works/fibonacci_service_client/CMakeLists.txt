cmake_minimum_required(VERSION 3.5)
project(fibonacci_action_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(fibonacci_action_client src/fibonacci_action_client.cpp)
ament_target_dependencies(fibonacci_action_client rclcpp rclcpp_action example_interfaces)

install(TARGETS
  fibonacci_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package() 
