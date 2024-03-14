cmake_minimum_required(VERSION 3.8)
project(roboarm_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(roboarm_remote REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmTask.action"
)

add_executable(arm_task_server src/arm_task_server.cpp)
ament_target_dependencies(arm_task_server rclcpp rclcpp_action roboarm_remote moveit_ros_planning_interface)

install(TARGETS
  arm_task_server
  DESTINATION lib/${PROJECT_NAME}
  )
ament_package()
