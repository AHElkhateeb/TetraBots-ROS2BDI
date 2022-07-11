cmake_minimum_required(VERSION 3.5)
project(airbus_demo)


# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_skills REQUIRED)
find_package(ros2_bdi_core REQUIRED)


set(bdi_act_client_dep
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  ros2_bdi_core
)

set(sensors_dep
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  ros2_bdi_core
)

add_executable(moveto src/transporting_agent/actions/moveto.cpp)
ament_target_dependencies(moveto ${bdi_act_client_dep})

add_executable(charge src/transporting_agent/actions/charge.cpp)
ament_target_dependencies(charge ${bdi_act_client_dep})

add_executable(pickup src/transporting_agent/actions/pickup.cpp)
ament_target_dependencies(pickup ${bdi_act_client_dep})

add_executable(drop src/transporting_agent/actions/drop.cpp)
ament_target_dependencies(drop ${bdi_act_client_dep})

add_executable(change_tool src/transporting_agent/actions/change_tool.cpp)
ament_target_dependencies(change_tool ${bdi_act_client_dep})

install(TARGETS
  moveto
  charge
  pickup
  drop
  change_tool
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
