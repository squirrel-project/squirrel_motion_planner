#include <ros/ros.h>
#include <ros/package.h>

#include <squirrel_8dof_visualizer/squirrel_8dof_visualizer.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "squirrel_8dof_visualizer_node");
  ros::NodeHandle nh("~");

  SquirrelMotionPlanning::Visualizer* visualizer = new SquirrelMotionPlanning::Visualizer("robot_description_vis");

  delete visualizer;

  return 0;
}
