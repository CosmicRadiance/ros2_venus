cmake_minimum_required(VERSION 3.5)
project(venus_description)

if( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
	set(BUILD_PLATFORM
		linux_x86_64)
else()
	set(BUILD_PLATFORM
		windows_x64)
endif()

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/include
)

add_executable(venus_control_node src/venus_control.cpp)

ament_target_dependencies(venus_control_node
  rclcpp
  sensor_msgs)

target_link_libraries(venus_control_node
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/libActuatorController.so
)

install(TARGETS venus_control_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  config gazebo launch meshes ros2_control urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
