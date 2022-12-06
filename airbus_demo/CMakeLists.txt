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
find_package(ros2_bdi_behaviours REQUIRED)
find_package(ros2_bdi_core REQUIRED)
find_package(bridge_interfaces REQUIRED)


set(bdi_act_client_dep
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  ros2_bdi_core
  bridge_interfaces
  ros2_bdi_behaviours
)

set(sensors_dep
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  ros2_bdi_core
  bridge_interfaces
)

#Transporting Agent
#Actions
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

add_executable(moveto_cooperatively src/transporting_agent/actions/moveto_cooperatively.cpp)
ament_target_dependencies(moveto_cooperatively ${bdi_act_client_dep})

add_executable(drop_cooperatively src/transporting_agent/actions/drop_cooperatively.cpp)
ament_target_dependencies(drop_cooperatively ${bdi_act_client_dep})

add_executable(pickup_cooperatively src/transporting_agent/actions/pickup_cooperatively.cpp)
ament_target_dependencies(pickup_cooperatively ${bdi_act_client_dep})

#Sensors
add_executable(battery_sensor src/transporting_agent/sensors/battery_sensor.cpp)
ament_target_dependencies(battery_sensor ${sensors_dep})

add_executable(position_sensor src/transporting_agent/sensors/position_sensor.cpp)
ament_target_dependencies(position_sensor ${sensors_dep})

add_executable(tool_sensor src/transporting_agent/sensors/tool_sensor.cpp)
ament_target_dependencies(tool_sensor ${sensors_dep})

#Organizing Agent
#Actions
add_executable(ask_for_transportation src/organizing_agent/actions/ask_for_transportation.cpp)
ament_target_dependencies(ask_for_transportation ${bdi_act_client_dep})
  
#Sensors
add_executable(payload_in_sensor src/organizing_agent/sensors/payload_in_sensor.cpp)
ament_target_dependencies(payload_in_sensor ${sensors_dep})

add_executable(payload_should_be_in_sensor src/organizing_agent/sensors/payload_should_be_in_sensor.cpp)
ament_target_dependencies(payload_should_be_in_sensor ${sensors_dep})

install(TARGETS
  moveto
  moveto_cooperatively
  charge
  pickup
  pickup_cooperatively
  drop
  drop_cooperatively
  change_tool
  battery_sensor
  position_sensor
  tool_sensor
  ask_for_transportation
  payload_in_sensor
  payload_should_be_in_sensor
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
