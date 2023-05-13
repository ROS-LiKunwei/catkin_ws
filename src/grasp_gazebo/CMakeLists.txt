cmake_minimum_required(VERSION 3.0.2)
project(grasp_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_PREFIX_PATH /home/lkw/catkin_ws/src/moveit ${CMAKE_PREFIX_PATH})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)


link_directories(${MOVEIT_LIBRARIES})






catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES grasp_gazebo
#  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS  moveit_core moveit_ros_planning moveit_ros_planning_interface
)


include_directories(
  include
  SYSTEM
  third_party
  ${catkin_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)




add_executable(grasp src/grasp.cpp)


target_link_libraries(grasp
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)


