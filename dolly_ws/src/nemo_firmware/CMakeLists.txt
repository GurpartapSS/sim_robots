cmake_minimum_required(VERSION 3.8)
project(nemo_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

pkg_check_modules(SERIAL libserial)
add_library(${PROJECT_NAME} SHARED
src/nemo_interface.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  "${SERIAL_INCLUDE_DIRS}"
)

target_link_libraries(${PROJECT_NAME} "${SERIAL_LDFLAGS}")
ament_target_dependencies(${PROJECT_NAME} 
hardware_interface
rclcpp 
rclcpp_lifecycle
pluginlib)

add_executable(simpleSerialPub src/arduino_publisher_test.cpp)
ament_target_dependencies(simpleSerialPub rclcpp std_msgs)
target_include_directories(simpleSerialPub PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simpleSerialPub "${SERIAL_LDFLAGS}")

add_executable(simpleSerialSub src/arduino_subscriber_test.cpp)
ament_target_dependencies(simpleSerialSub rclcpp std_msgs)
target_include_directories(simpleSerialSub PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simpleSerialSub "${SERIAL_LDFLAGS}")

pluginlib_export_plugin_description_file(hardware_interface nemo_interface.xml)

install(TARGETS
${PROJECT_NAME}
DESTINATION lib)



install(TARGETS
simpleSerialPub
simpleSerialSub
ARCHIVE DESTINATION lib
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})


ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
