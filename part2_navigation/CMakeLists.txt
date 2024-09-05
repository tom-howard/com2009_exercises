cmake_minimum_required(VERSION 3.8)
project(part2_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_executable src/minimal_cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/odom_subscriber.py
  scripts/move_circle.py
  scripts/move_square.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
