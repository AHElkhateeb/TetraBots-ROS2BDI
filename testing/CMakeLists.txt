cmake_minimum_required(VERSION 3.5)
project(testing)

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
find_package(bridge_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp bridge_interfaces)

install(TARGETS
  subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
