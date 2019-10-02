cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_rbp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_msgs
    tf
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller.cpp)
add_dependencies(motor_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(motor_controller ${catkin_LIBRARIES} /opt/EposCmdLib_6.3.1.0/lib/armv7hf/libEposCmd.so.6.3.1.0)

