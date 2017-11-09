cmake_minimum_required(VERSION 2.8.3)
project(validity_checker)

find_package(catkin REQUIRED COMPONENTS
  kuka_motion_control
  planner_data_structures
  #moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  #tf
  #moveit_msgs
  
  octomap_msgs
  octomap_server
  kdl_parser
  fcl
)

find_package(octomap REQUIRED)

find_package(orocos_kdl REQUIRED)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES validity_checker
   CATKIN_DEPENDS kuka_motion_control planner_data_structures roscpp rospy std_msgs octomap_msgs octomap_server  kdl_parser  fcl # moveit_msgs tf
   DEPENDS octomap orocos_kdl
)

###################
## Project Files ##
###################

set(validity_checker_files

#CPP Files

src/feasibility_checker.cpp

#Header files (added only for visualization in QtCreator)
include/validity_checker/feasibility_checker.h

)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${validity_checker_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})