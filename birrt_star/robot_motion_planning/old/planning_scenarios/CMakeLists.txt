cmake_minimum_required(VERSION 2.8.3)
project(planning_scenarios)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rrt_star_algorithm
  birrt_star_algorithm
  planning_world_builder
)

#if(birrt_star_algorithm_FOUND)
#  message (STATUS "birrt_star_algorithm found")
#else(birrt_star_algorithm_FOUND)
#  message (STATUS "birrt_star_algorithm not found")
#endif()

catkin_package(
   CATKIN_DEPENDS roscpp rospy  planning_world_builder birrt_star_algorithm rrt_star_algorithm
   DEPENDS system_lib
)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Executable for running RRT-star Algorithm
add_executable(run_start_goal_config_generator src/generateStartGoalConfigs.cpp)
add_executable(run_block_planning src/run_block_scenario.cpp)
add_executable(run_empty_world_planning src/run_empty_world_scenario.cpp)
add_executable(run_cart_narrow_corridor_planning src/run_cart_narrow_corridor_scenario.cpp)
add_executable(run_cart_three_gates_planning src/run_cart_three_gates_scenario.cpp)
add_executable(run_cart_tunnel_planning src/run_cart_tunnel_scenario.cpp)
add_executable(run_cart_two_rooms_office_planning src/run_cart_two_rooms_office_scenario.cpp)
add_executable(run_glass_delivery_planning src/run_glass_delivery_scenario.cpp)
add_executable(run_cart_parking_slot_planning src/run_cart_parking_slot_scenario.cpp)
add_executable(run_rack_planning src/run_rack_scenario.cpp)
add_executable(run_door_planning src/run_door_scenario.cpp)
#Run planner comparison on block scenario
add_executable(run_planner_comparison_block_planning src/run_planner_comparison_block_scenario.cpp)
add_executable(run_planner_comparison_glass_planning src/run_planner_comparison_glass_scenario.cpp)

target_link_libraries(run_start_goal_config_generator ${catkin_LIBRARIES})
target_link_libraries(run_block_planning ${catkin_LIBRARIES})
target_link_libraries(run_empty_world_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_narrow_corridor_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_three_gates_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_tunnel_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_two_rooms_office_planning ${catkin_LIBRARIES})
target_link_libraries(run_glass_delivery_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_parking_slot_planning ${catkin_LIBRARIES})
target_link_libraries(run_rack_planning ${catkin_LIBRARIES})
target_link_libraries(run_door_planning ${catkin_LIBRARIES})
target_link_libraries(run_planner_comparison_block_planning ${catkin_LIBRARIES})
target_link_libraries(run_planner_comparison_glass_planning ${catkin_LIBRARIES})

install(TARGETS run_start_goal_config_generator run_block_planning run_empty_world_planning run_cart_narrow_corridor_planning run_cart_three_gates_planning run_cart_tunnel_planning run_cart_two_rooms_office_planning run_glass_delivery_planning run_cart_parking_slot_planning run_rack_planning run_door_planning run_planner_comparison_block_planning run_planner_comparison_glass_planning RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})