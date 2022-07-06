cmake_minimum_required(VERSION 3.5)
project(bridge_interfaces)

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

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
   "msg/BotStatus.msg"
   "msg/HotLine.msg"
   "msg/StillAlive.msg"
   "msg/T265PosAndOri.msg"
   "msg/TeamCommand.msg"
   "msg/JobDefinition.msg"
   "msg/TeamDefinition.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

# Install the mapping rules for parameter name matches within the message.
install(
  FILES rosbridge_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})
