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
   CATKIN_DEPENDS roscpp rospy rviz_robot_marker std_msgs tf sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/squirrel_8dof_visualizer_node.cpp src/squirrel_8dof_visualizer.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})