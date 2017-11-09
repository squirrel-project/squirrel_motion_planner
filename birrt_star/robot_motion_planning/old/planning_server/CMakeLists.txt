cmake_minimum_required(VERSION 2.8.3)
project(planning_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  planner_msgs
  rrt_star_algorithm
  birrt_star_algorithm
  #planning_world_builder
  #motion_trajectory_execution
  eigen_conversions
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES planning_server
   CATKIN_DEPENDS roscpp rospy std_msgs planner_msgs birrt_star_algorithm rrt_star_algorithm eigen_conversions
)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

set(PLANNING_SERVER_LIBRARIES
    #CPP Files
    src/planning_server.cpp
    src/planning_control_center.cpp
    
    #Header files (added only for visualization in QtCreator)
    include/planning_server/planning_server.h
    include/planning_server/planning_control_center.h
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${PLANNING_SERVER_LIBRARIES})

add_executable(planning_service_node src/planning_service_node.cpp)
target_link_libraries(planning_service_node ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(base_planning_service src/base_motion_planning_service.cpp)
#target_link_libraries(base_planning_service ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS planning_service_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})