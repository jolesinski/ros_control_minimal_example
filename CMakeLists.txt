cmake_minimum_required(VERSION 2.8.3)
project(ros_control_minimal_example)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    roscpp
    trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_robot_hw src/my_robot.cpp)
add_dependencies(my_robot_hw
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(my_robot_hw
  ${catkin_LIBRARIES}
)
