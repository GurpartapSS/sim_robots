cmake_minimum_required(VERSION 3.8)
project(robo_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED COMPONENTS
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
)

install(
  DIRECTORY description launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
