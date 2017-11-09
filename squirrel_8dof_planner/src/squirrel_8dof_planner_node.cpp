#include <ros/ros.h>
#include <ros/package.h>

#include <squirrel_8dof_planner/squirrel_8dof_planner.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "squirrel_8dof_planner_node");

  SquirrelMotionPlanner::Planner planner;

  ros::spin();

  return 0;
}

