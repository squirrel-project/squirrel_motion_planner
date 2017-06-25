#ifndef SQUIRREL_8DOF_VISUALIZER_H_
#define SQUIRREL_8DOF_VISUALIZER_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <rviz_robot_marker/RvizRobotMarkerPublisher.h>

#include <vector>
#include <string>
#include <stdint.h>

namespace SquirrelMotionPlanning
{
typedef int32_t Int;     ///< 32-bit signed integer definition.
typedef uint32_t UInt;     ///< 32-bit unsigned integer definition.
typedef double Real;     ///< 64-bit floating point definition.

/**
 * @brief Visualizer class, used for visualizing 8dof trajectories of the squirrel robot.
 * The node subscribes to a multi array topic that contains the full robot trajectory and displays the full trajectory
 * as a ghost model ghost model that performs the trajectory perfectly.
 */
class Visualizer
{
public:
  /**
   * @brief Constructor. Initializes all topics, variables and launches the visualizer.
   */
  Visualizer(const std::string &robotDescriptionTopic);

private:
  ros::NodeHandle nh;     ///< ROS node handle with global namespace.
  ros::NodeHandle nhPrivate;     ///< ROS node handle with namespace relative to current node.
  ros::Subscriber subscriberTrajectory;     ///< ROS subscriber. Subscribes to a Float64MultiArray that contains a vector of 8D poses.
  ros::Subscriber subscriberRate;     ///< ROS subscriber. Subscribes to a Float64 and sets the publishing speed of the trajectory.
  ros::Subscriber subscriberVisibility;     ///< ROS subscriber. Subscribes to a Bool and sets the visiblity of the trajectory visualization.
  rviz_robot_marker::RvizRobotMarkerPublisher robotMarkerPublisher;     ///< Publisher class, takes care of direct visualization to Rviz.
  UInt robotID;     ///< Robot ID needed to communicate with robotMarkerPublisher.

  std::vector<std::vector<Real> > poses;     ///< Vector to which all 8D poses are saved, once a new trajectory is received.
  UInt poseCurrent;     ///< Index of the current pose to be visualized.

  tf::StampedTransform transformBase;     ///< The full base transform of the robot, sent to robotMarkerPublisher.
  tf::Vector3 transformBaseTranslation;     ///< Base translation, used to set translation of transformBase.
  tf::Matrix3x3 transformBaseRotation;     ///< Base rotation, used to set rotation of transformBase.
  sensor_msgs::JointStatePtr jointStatesArm;     ///< The full set of arm joint angles, sent to robotMarkerPublisher.

  ros::Rate rate;     ///< Current publishing rate, set in subscriberRateHandler.
  bool visible;     ///< Set in subscriberVisibilityHandler, indicates if the visualization is currently visible.
  bool finalPoseState;     ///< True if the final pose is currently being shown.
  ros::WallTime finalPoseTime;     ///< Time at which the last final pose has been reached.

  /**
   * @brief Main loop running at rate. Sets current robot pose and publishes it using robotMarkerPublisher.
   */
  void run();

  /**
   * @brief Handler for the robot trajectory. Checks the dimension of the multi array and saves it to poses.
   * @param msg Contains the full robot trajectory represented as vector of 8D poses.
   */
  void subscriberTrajectoryHandler(const std_msgs::Float64MultiArray &msg);

  /**
    * @brief Handler for the visualization rate. Sets rate and changes the sleep time between visualization of poses in the main loop.
    * @param msg Contains the new rate.
    */
  void subscriberRateHandler(const std_msgs::Float64 &msg);

  /**
   * @brief Handler for the visibility of the visualization. Sets visible and changes the visibility of the ghost model in Rviz.
   * @param msg Contains the visiblity state.
   */
  void subscriberVisibilityHandler(const std_msgs::Bool &msg);

}; //Visualizer


} //namespace SquirrelMotionPlanning

#endif //SQUIRREL_8DOF_VISUALIZER_H_

