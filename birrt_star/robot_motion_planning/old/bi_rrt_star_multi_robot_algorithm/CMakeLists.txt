cmake_minimum_required(VERSION 2.8.3)
project(bi_rrt_star_multi_robot_algorithm)

find_package(catkin REQUIRED COMPONENTS
  kuka_motion_control
  moveit_ros_planning_interface
  planner_data_structures
  planning_heuristics
  planning_world_builder
  validity_checker
  roscpp
  rospy
  std_msgs
)

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
   LIBRARIES bi_rrt_star_multi_robot_algorithm
   CATKIN_DEPENDS kuka_motion_control planning_world_builder planner_data_structures validity_checker moveit_ros_planning_interface planning_heuristics roscpp rospy std_msgs
   DEPENDS system_lib
)

###################
## Project Files ##
###################

set(bi_rrt_star_multi_robot_algorithm_files

#CPP Files
#src/run_rrt_star_algorithm.cpp
src/birrt_star_multi_robot.cpp

#Header files (added only for visualization in QtCreator)
include/bi_rrt_star_multi_robot_algorithm/birrt_star_multi_robot.h

)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${bi_rrt_star_multi_robot_algorithm_files})

add_executable(bi_rrt_star_multi_robot_example src/run_birrt_star_multi_robot_example.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(bi_rrt_star_multi_robot_example ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS bi_rrt_star_multi_robot_example RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})