cmake_minimum_required(VERSION 2.8.3)
project(planner_data_structures)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES planner_data_structures
   CATKIN_DEPENDS roscpp rospy std_msgs
)

set(planner_data_structures_files

#CPP Files
src/data_structs.cpp

#Header files (added only for visualization in QtCreator)
include/planner_data_structures/data_structs.h

)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${planner_data_structures_files})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

