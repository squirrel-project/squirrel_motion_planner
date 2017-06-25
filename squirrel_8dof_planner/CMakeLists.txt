cmake_minimum_required(VERSION 2.8.3)
project(squirrel_8dof_planner)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  sensor_msgs
  birrt_star_algorithm
  moveit_ros_planning
  moveit_msgs
  interactive_markers
)

find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs tf sensor_msgs birrt_star_algorithm moveit_ros_planning moveit_msgs interactive_markers
)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/${PROJECT_NAME}.cpp src/${PROJECT_NAME}_structures.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
