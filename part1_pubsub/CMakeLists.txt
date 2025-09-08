cmake_minimum_required(VERSION 3.8)
project(part1_pubsub_jazzy)

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

# Install Python modules
ament_python_install_package(${PROJECT_NAME}_modules)

# Install Python executables
install(PROGRAMS
  scrips/publisher.py
  scrips/subscriber.py
  scrips/custom_msg_publisher.py
  scrips/custom_msg_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Example.msg" 
)

ament_package()
