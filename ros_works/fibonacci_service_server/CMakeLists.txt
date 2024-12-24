cmake_minimum_required(VERSION 3.5)
project(fibonacci_action_server)

# Убедимся, что используем стандарт C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Найти зависимости ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

# Укажите исходные файлы
add_executable(fibonacci_action_server src/fibonacci_action_server.cpp)

# Подключаем зависимости
ament_target_dependencies(fibonacci_action_server
  rclcpp
  rclcpp_action
  example_interfaces
)

# Указываем, где должен быть установлен исполняемый файл
install(TARGETS
  fibonacci_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Завершаем настройку пакета
ament_package()
