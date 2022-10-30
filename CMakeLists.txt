cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)

add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  visualization_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
#find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_planner
  #${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_executable(planner_node src/node.cpp)
add_dependencies(planner_node ${dwa_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)