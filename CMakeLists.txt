cmake_minimum_required(VERSION 3.10)
project(calixto-ros-bot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED) # This must be found for the command to work
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 2. Build the hardware library
add_library(calixto_hardware SHARED
  hardware/diffbot_system.cpp
)

target_include_directories(calixto_hardware PRIVATE
  hardware/include
)

# Use ament_target_dependencies to properly link everything
ament_target_dependencies(calixto_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 3. Export the plugin (The command that was failing)
# NOTE: This command name is case sensitive and requires pluginlib to be found
pluginlib_export_plugin_description_file(hardware_interface calixto-ros-bot.xml)

# 4. Install
install(TARGETS calixto_hardware
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the export XML file so it can be found at runtime
install(FILES calixto-ros-bot.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
