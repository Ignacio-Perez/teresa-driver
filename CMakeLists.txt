cmake_minimum_required(VERSION 2.8.3)

project(teresa_driver)

add_definitions(-std=c++11)
add_definitions(-Wall)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  Batteries.msg
  Stalk.msg
  StalkRef.msg
  Volume.msg
  Buttons.msg
  Temperature.msg
  Diagnostics.msg
  CmdVelRaw.msg
  WheelVels.msg
)

add_service_files(
  FILES
  Get_DCDC.srv
  Set_DCDC.srv
  Teresa_leds.srv
)

generate_messages(
   DEPENDENCIES
   nav_msgs   sensor_msgs   std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES teresa_driver
 CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf message_runtime
 DEPENDS system_lib
)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teresa_node src/teresa_node.cpp)
add_dependencies(teresa_node teresa_driver_gencpp teresa_driver_generate_messages_cpp)
add_executable(teresa_teleop_joy src/teresa_teleop_joy.cpp)
add_dependencies(teresa_teleop_joy teresa_driver_gencpp teresa_driver_generate_messages_cpp)
add_executable(teresa_node_calib src/teresa_node_calib.cpp)
add_dependencies(teresa_node_calib teresa_driver_gencpp teresa_driver_generate_messages_cpp)


target_link_libraries(teresa_node
   ${catkin_LIBRARIES}
)

target_link_libraries(teresa_teleop_joy
   ${catkin_LIBRARIES}
)

target_link_libraries(teresa_node_calib
   ${catkin_LIBRARIES}
)


