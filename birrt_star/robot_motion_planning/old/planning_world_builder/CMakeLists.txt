cmake_minimum_required(VERSION 2.8.3)
project(planning_world_builder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_ros_planning
  kuka_motion_control
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES planning_world_builder
   CATKIN_DEPENDS roscpp rospy std_msgs moveit_ros_planning kuka_motion_control
   DEPENDS system_lib
)

###################
## Project Files ##
###################

set(planning_world_builder_files

#CPP Files
src/planning_world_builder.cpp

#Header files (added only for visualization in QtCreator)
include/planning_world_builder/planning_world_builder.h

)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

# Build Libraries
add_library(${PROJECT_NAME} ${planning_world_builder_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#Executable for running Planning World Builder Node
#add_executable(rrt_star_node src/run_rrt_star_algorithm.cpp)

# add_dependencies(rrt_star_algorithm_node rrt_star_algorithm_generate_messages_cpp)

#target_link_libraries(rrt_star_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})