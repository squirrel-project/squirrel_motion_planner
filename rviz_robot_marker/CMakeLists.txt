cmake_minimum_required(VERSION 2.8.3)
project(rviz_robot_marker)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  urdf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS kdl_parser roscpp tf tf_conversions urdf visualization_msgs sensor_msgs std_msgs
)

string(REPLACE "." ";" VERSION_LIST ${${PROJECT_NAME}_VERSION})
list(GET VERSION_LIST 0 ${PROJECT_NAME}_VERSION_MAJOR)
list(GET VERSION_LIST 1 ${PROJECT_NAME}_VERSION_MINOR)
list(GET VERSION_LIST 2 ${PROJECT_NAME}_VERSION_PATCH)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/RvizRobotMarkerPublisher.cpp) 
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${${PROJECT_NAME}_VERSION}
    SOVERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(publish_marker src/publish_marker.cpp)
add_dependencies(publish_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_marker ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}   
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
