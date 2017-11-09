cmake_minimum_required(VERSION 2.8.3)
project(planner_msgs)

#List to make rest of code more readable
set(MESSAGE_DEPENDENCIES
    std_msgs
    geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${MESSAGE_DEPENDENCIES}
  message_generation
)

add_message_files(
    FILES
    array1d_double.msg
    array2d_double.msg
)

add_service_files(
   FILES
   #General Planner Services
   generate_start_goal_config.srv
   planning_scene_info.srv
   #reset_planner_to_default.srv

   #Planner Setup Services
   set_edge_costs.srv

   #Run Planner Services (Simulation)
   run_planner_configs.srv
   run_planner_config_file.srv
   run_planner_poses.srv
   run_planner_start_config_goal_pose.srv

   #Run Planner Services (Real Robot)
   #run_planner_real_robot_goal_pose.srv
   #run_planner_real_robot_goal_config.srv

 )

generate_messages(
    DEPENDENCIES
    ${MESSAGE_DEPENDENCIES}
 )

catkin_package(
    CATKIN_DEPENDS message_runtime)

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

