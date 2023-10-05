cmake_minimum_required(VERSION 3.0.2)
project(moveit_command)


find_package(catkin REQUIRED COMPONENTS
  roscpp

  cmake_modules
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_msgs
  adlink_msg
)


catkin_package(
CATKIN_DEPENDS 

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#windows command




add_executable(moveit_command src/Command_Execute.cpp)
target_link_libraries(moveit_command ${catkin_LIBRARIES})




