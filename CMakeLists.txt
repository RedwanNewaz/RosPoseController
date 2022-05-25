cmake_minimum_required(VERSION 2.8.3)
project(pose_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_controller
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(Eigen3 REQUIRED)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME}_node src/pose_controller_node.cpp include/state.h include/pid.h)


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
         Eigen3::Eigen
 )

