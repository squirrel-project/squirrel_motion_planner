cmake_minimum_required(VERSION 2.8.3)
project(kuka_motion_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kdl_parser
  moveit_core
  moveit_ros_planning
  srdfdom
  tf
)

find_package(orocos_kdl REQUIRED)

#   std_msgs

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs kdl_parser moveit_core moveit_ros_planning srdfdom tf
  DEPENDS system_lib orocos_kdl
)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${orocos_kdl_LIBRARY_DIRS}
)

#Project Files
set(kuka_motion_control_files

#CPP Files
src/kdl_kuka_model.cpp
src/control_laws.cpp

#Header files (added only for visualization in QtCreator)
include/kuka_motion_control/kdl_kuka_model.h
include/kuka_motion_control/control_laws.h
)

# Declare a cpp library
add_library(${PROJECT_NAME} ${kuka_motion_control_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#Test files (for arm motion control and reaching task )
add_executable(kuka_motion_control_node src/motion_control_node.cpp)
target_link_libraries(kuka_motion_control_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS kuka_motion_control_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})