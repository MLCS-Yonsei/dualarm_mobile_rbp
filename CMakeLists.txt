cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_rbp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES} /opt/EposCmdLib_6.3.1.0/lib/armv7hf/libEposCmd.so.6.3.1.0)


