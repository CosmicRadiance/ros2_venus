cmake_minimum_required(VERSION 3.5)
project(venus_hardware)

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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# include local directories and dependencies that are not ament packages
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/include
)

# COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/venus_system_position_only.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

# link libraries that are not ament packages
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/libActuatorController.so
)

pluginlib_export_plugin_description_file(hardware_interface venus_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
