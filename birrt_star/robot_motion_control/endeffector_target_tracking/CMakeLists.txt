cmake_minimum_required(VERSION 2.8.3)
project(endeffector_target_tracking)

find_package(catkin REQUIRED COMPONENTS
  kuka_motion_control
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

#   std_msgs

catkin_package(
#   INCLUDE_DIRS include
#  CATKIN_DEPENDS kuka_motion_control roscpp rospy std_msgs
CATKIN_DEPENDS roscpp visualization_msgs)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_control_node src/target_control_node.cpp)
add_executable(lbr_robot_controller src/lbr_robot_controller.cpp)

target_link_libraries(target_control_node ${catkin_LIBRARIES})
target_link_libraries(lbr_robot_controller ${catkin_LIBRARIES})

add_dependencies(target_control_node ${catkin_EXPORTED_TARGETS})
add_dependencies(lbr_robot_controller ${catkin_EXPORTED_TARGETS})
install(TARGETS target_control_node lbr_robot_controller RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})