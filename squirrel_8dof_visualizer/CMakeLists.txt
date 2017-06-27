cmake_minimum_required(VERSION 2.8.3)
project(squirrel_8dof_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz_robot_marker
  std_msgs
  tf
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES squirrel_8dof_visualizer
   CATKIN_DEPENDS roscpp rospy rviz_robot_marker std_msgs tf sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/squirrel_8dof_visualizer_node.cpp src/squirrel_8dof_visualizer.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
