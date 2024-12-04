cmake_minimum_required(VERSION 3.8)
project(inverse_kine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(roboarm_remote REQUIRED)
find_package(inverse_kine REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartisanCoord.msg"
)

add_executable(xyzCoord src/xyzCoordinatesNode.cpp)
ament_target_dependencies(xyzCoord rclcpp roboarm_remote inverse_kine rclcpp_action)

add_executable(jointStatesMonitor src/jointStatesMonitor.cpp)
ament_target_dependencies(jointStatesMonitor rclcpp sensor_msgs)

install(TARGETS
xyzCoord
jointStatesMonitor
DESTINATION lib/${PROJECT_NAME}
)


ament_package()
