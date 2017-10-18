cmake_minimum_required(VERSION 2.8.3)
project(birrt_star_algorithm)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  planner_data_structures
  planning_heuristics
  planning_world_builder
  validity_checker
  kuka_motion_control
  roscpp
  rospy
  std_msgs
  #robot_interface_definition
  tf
  rviz_visual_tools
)

#Find OpenMP Package
#find_package(OpenMP)

#For including robot_interface_definition header files
#find_package(robot_interface_definition REQUIRED)

#if(validity_checker_FOUND)
#  message (STATUS "Validity checker found")
#else(validity_checker_FOUND)
#  message (STATUS "Validity checker not found")
#endif()

#check for OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  if(MSVC90 OR MSVC10)
    if(MSVC90)
      set(OPENMP_DLL VCOMP90)
    elseif(MSVC10)
      set(OPENMP_DLL VCOMP100)
    endif(MSVC90)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
  endif(MSVC)
else(OPENMP_FOUND)
  message (STATUS "OpenMP not found")
endif()

if (MSVC)
  Set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /O2 ${SSE_FLAGS}")
else (MSVC)
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS} ${SSE_FLAGS}")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g ${SSE_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE " ${CMAKE_CXX_FLAGS} -O3 ${SSE_FLAGS}")
endif (MSVC)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES birrt_star_algorithm
   CATKIN_DEPENDS kuka_motion_control planning_world_builder planner_data_structures validity_checker moveit_ros_planning_interface planning_heuristics roscpp rospy std_msgs tf
   DEPENDS system_lib
)

###################
## Project Files ##
###################

set(birrt_star_algorithm_files

#CPP Files
#src/run_rrt_star_algorithm.cpp
src/birrt_star.cpp

#Header files (added only for visualization in QtCreator)
include/birrt_star_algorithm/birrt_star.h

)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${robot_interface_definition_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${birrt_star_algorithm_files})

#Executable for running RRT-star Algorithm (Omnirob)
add_executable(birrt_star_omnirob_lbr_node src/run_birrt_star_algorithm_omnirob_lbr.cpp)
add_executable(birrt_star_lbr_only_node src/run_birrt_star_algorithm_lbr_only.cpp)
add_executable(birrt_star_omnirob_base_node src/run_birrt_star_algorithm_omnirob_base.cpp)

#Executable for running RRT-star Algorithm (Robotino)
add_executable(birrt_star_robotino_node src/run_birrt_star_algorithm_robotino.cpp)

#Executable for running RRT-star Algorithm (PR2)
add_executable(birrt_star_pr2_base_node src/run_birrt_star_algorithm_pr2_base.cpp)
add_executable(birrt_star_pr2_base_arm_node src/run_birrt_star_algorithm_pr2_base_arm.cpp)

# add_dependencies(rrt_star_algorithm_node rrt_star_algorithm_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_omnirob_lbr_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_lbr_only_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_omnirob_base_node ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(birrt_star_robotino_node ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(birrt_star_pr2_base_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_pr2_base_arm_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS birrt_star_omnirob_lbr_node birrt_star_lbr_only_node birrt_star_omnirob_base_node birrt_star_robotino_node birrt_star_pr2_base_node birrt_star_pr2_base_arm_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})