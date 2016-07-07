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
  Diagnosis.msg
)

add_service_files(
  FILES
  Teresa_DCDC.srv
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
add_dependencies(teresa_node teresa_ros_driver_gencpp)
add_executable(teresa_teleop_joy src/teresa_teleop_joy.cpp)
add_dependencies(teresa_teleop_joy teresa_ros_driver_gencpp)

target_link_libraries(teresa_node
   ${catkin_LIBRARIES}
)

target_link_libraries(teresa_teleop_joy
   ${catkin_LIBRARIES}
)