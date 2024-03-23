cmake_minimum_required(VERSION 3.8)
project(opencv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_executable(imageViewerNode src/imageViewerNode.cpp)
ament_target_dependencies(imageViewerNode rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
imageViewerNode
  DESTINATION lib/${PROJECT_NAME}
  )


ament_package()
