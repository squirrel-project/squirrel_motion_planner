cmake_minimum_required(VERSION 2.8.3)
project(birrt_star_algorithm)

find_package(catkin REQUIRED COMPONENTS
  planner_data_structures
  planning_heuristics
  kuka_motion_control
  roscpp
  rospy
  std_msgs
  octomap_msgs
  octomap_server
  kdl_parser
  fcl
)

find_package(octomap REQUIRED)

find_package(orocos_kdl REQUIRED)

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
   CATKIN_DEPENDS kuka_motion_control planner_data_structures planning_heuristics roscpp rospy std_msgs octomap_msgs octomap_server kdl_parser fcl
   DEPENDS system_lib octomap orocos_kdl
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
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${birrt_star_algorithm_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${orocos_kdl_LIBRARIES})
