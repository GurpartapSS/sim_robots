cmake_minimum_required(VERSION 3.8)
project(examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(simpleLifeCycle src/simple_lifecycle.cpp)
ament_target_dependencies(simpleLifeCycle rclcpp std_msgs rclcpp_lifecycle)

install(TARGETS
  simpleLifeCycle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
