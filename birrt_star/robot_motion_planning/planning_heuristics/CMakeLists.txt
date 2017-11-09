cmake_minimum_required(VERSION 2.8.3)
project(planning_heuristics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES planning_heuristics
   CATKIN_DEPENDS std_msgs)

## Project Files ##

set(planning_heuristics_files

#CPP Files
src/distance_heuristics.cpp

#Header files (added only for visualization in QtCreator)
include/planning_heuristics/distance_heuristics.h

)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}  ${planning_heuristics_files})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})