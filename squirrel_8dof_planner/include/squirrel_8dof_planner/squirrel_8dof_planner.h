#ifndef SQUIRREL_8DOF_PLANNER_H_
#define SQUIRREL_8DOF_PLANNER_H_

#include <squirrel_8dof_planner/squirrel_8dof_planner_structures.h>

#include <string>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit_msgs/PlanningScene.h>
#include <interactive_markers/interactive_marker_server.h>
#include <birrt_star_algorithm/birrt_star.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

namespace SquirrelMotionPlanner
{

/**
 * @brief Planner class, used for 8dof planning from the current robot pose to a desired goal pose or end effector location.
 * Uses a 2-stage planning approach that first searches for a 2D A* path to the goal, shortens the path by a fixed distance,
 * and then smoothens the path. The last pose is then used to find a full 8D plan that includes the arm movement.
 */
class Planner
{
  /*
   * ROS message handling
   */
  ros::NodeHandle nh;     ///< ROS node handle with global namespace.
  ros::NodeHandle nhPrivate;     ///< ROS node handle with namespace relative to current node.
  ros::Publisher publisherPlanningScene;     ///< ROS publisher. Publishes the octree as a planning scene for moveit.
  ros::Publisher publisher2DPath;     ///< ROS publisher. Publishes the 2D projection of the 8D trajectory.
  ros::Publisher publisherTrajectory;     ///< ROS publisher. Publishes the full 8D trajectory.
  ros::Subscriber subscriberBase;     ///< ROS subscriber. Subscribes to /base/joint_angles.
  ros::Subscriber subscriberArm;     ///< ROS subscriber. Subscribes to /arm_controller/joint_states.
  ros::Subscriber subscriberGoalEndEffector;     ///< ROS subscriber. Subscribes to goal_end_effector.
  ros::Subscriber subscriberGoalPose;     ///< ROS subscriber. Subscribes to goal_pose.
  ros::Subscriber subscriberGoalMarkerExecute;     ///< ROS subscriber. Subscribes to goal_marker_execute.
  interactive_markers::InteractiveMarkerServer interactiveMarkerServer;     ///< Server that commuincates with Rviz to receive 6D end effector poses.
  visualization_msgs::InteractiveMarker interactiveMarker;     ///< Interactive marker used by interactiveMarkerServer.
  std::vector<Real> poseGoalMarker;     ///< Current pose of the interactive marker in RViz.

  /*
   * General search settings
   */
  std::vector<Real> poseInit;     ///< Vector with the 5 joint angles of the retracted arm during 2D planning.
  Real distanceBirrtStarPlanning;     ///< Distance to the goal pose from where the 8D planning is performed.
  Real obstacleInflationRadius;     ///< Inflation radius around occupied cells in occupancyMap.

  /*
   *  Current robot and planning poses
   */
  std::vector<Real> poseCurrent;     ///< 8D vector with the current robot pose, given as [x, y, theta, arm1, arm2, arm3, arm4, arm5].
  std::vector<Real> poseGoal;      ///< 8D vector with the goal pose for the robot, given as [x, y, theta, arm1, arm2, arm3, arm4, arm5].
  std::vector<std::vector<Real> > poses;   ///< Vector of 8D poses that contains the last succesfully computed trajectory from poseCurrent to poseGoal.

  /*
   * Octomap
   */
  octomap::OcTree octree;     ///< Current octree, used for computing occupancyMap and during 8dof planning.
  std::vector<std::vector<bool> > occupancyMap;     ///< Current occupancy map, computed from octree and inflated by obstalceInflationRadius.
  Real octreeMinX;     ///< Minimum metric x-value of octree that is used for the 2D planning.
  Real octreeMinY;     ///< Minimum metric y-value of octree that is used for the 2D planning.
  Real octreeMinZ;     ///< Minimum metric z-value of octree that is used for the 2D planning.
  Real octreeMaxX;     ///< Maximum metric x-value of octree that is used for the 2D planning.
  Real octreeMaxY;     ///< Maximum metric y-value of octree that is used for the 2D planning.
  Real octreeMaxZ;     ///< Maximum metric z-value of octree that is used for the 2D planning.
  Int octreeKeyMinX;     ///< Minimum octree key x-value that is used for creating occupancyMap.
  Int octreeKeyMinY;     ///< Minimum octree key y-value that is used for creating occupancyMap.
  Int octreeKeyMinZ;     ///< Minimum octree key z-value that is used for creating occupancyMap.
  Int octreeKeyMaxX;     ///< Maximum octree key x-value that is used for creating occupancyMap.
  Int octreeKeyMaxY;     ///< Maximum octree key y-value that is used for creating occupancyMap.
  Int octreeKeyMaxZ;     ///< Maximum octree key z-value that is used for creating occupancyMap.

  /*
   * AStar
   */
  AStarPath2D AStarPath;     ///< Full A* grid path that is found during the 2D planning part.
  Cell2D AStarCellStart;     ///< Start cell for the current 2D path search.
  Cell2D AStarCellGoal;     ///< Goal cell for the current 2D path search.
  std::vector<std::vector<AStarNode> > AStarNodes;     ///< Structured map of A* nodes that is used during the path search.
  std::vector<Cell2D> openListNodes;     ///< Priority queue of grid cells on AStarNodes for the A* path search.
  Real AStarPathSmoothingFactor;     ///< Factor that indicates the smoothing behavior of the A* path; good value lies between 1.5 and 2.5.
  Real AStarPathSmoothingDistance;     ///< Maximum distance from intermediate corners that is used during the A* path smoothing.
  Real AStarPathSmoothedPointDistance;     ///< The approximate final distance between points of the smoothed A* path.

  /*
   * Bi2RRTStar
   */
  birrt_star_motion_planning::BiRRTstarPlanner birrtStarPlanner;     ///< Instance of the 8dof BiRRT* planner.
  Int birrtStarPlanningNumber;     ///< The current planning number of the BiRRT* planner; increases by 1 after every planning attempt.

public:

  /**
   * @brief Constructur initializes everything, mainly calls all initialize methods. All planning and communication is done via ROS.
   */
  Planner();

private:

  /**
   * @brief Loads all necessary parameters from the ROS parameter server and initializeses further internal parameters.
   */
  void initializeParameters();

  /**
   * @brief Advertises and subscribes to all topics used for getting the current robot pose and handle planning requests.
   */
  void initializeMessageHandling();

  /**
   * @brief Initializeses the map used for the 2D astar planning.
   */
  void initializeAStarPlanning();

  /**
   * @brief Initializses and interactive marker that can be used for planning to specific end effector poses in rviz.
   */
  void initializeInteractiveMarker();

  /**
   * @brief Publishes the loaded octomap to the planning_scene topic for collision checks of moveit used in birrtstar.
   */
  void loadOctomapToMoveit();

  /**
   * @brief Publishes a 2D path that follows the base of the full trajectory after planning.
   */
  void publish2DPath() const;

  /**
   * @brief Publishes a vector of subsequent poses from the current robot position to the requested goal after planning.
   */
  void publishTrajectory() const;

  /**
   * @brief Handler for the base position of the robot.
   * @param msg ROS message that contains the base pose information.
   */
  void subscriberBaseHandler(const sensor_msgs::JointState &msg);

  /**
   * @brief Handler for the arm joints of the robot.
   * @param msg ROS message that contains the joint angles of  the arm information.
   */
  void subscriberArmHandler(const sensor_msgs::JointState &msg);

  /**
   * @brief Handler for starting the planner to the requested end effector position.
   * @param msg ROS message that contains the 6D pose of the requested end effector goal position given in [x,y,z,R,P,Y].
   */
  void subscriberGoalEndEffectorHandler(const std_msgs::Float64MultiArray &msg);

  /**
   * @brief Used to signal a request for finding a trajectory to the current interactive marker pose for the endeffector.
   * @param msg Emtpy message used only as a signal.
   */
  void subscriberGoalMarkerExecuteHandler(const std_msgs::Empty &msg);

  /**
   * @brief Handler for starting the planner to the requested goal pose.
   * @param msg ROS message that contains the full 8D pose of the robot given as
   * [base_x, base_y, base_theta, arm_joint1, arm_joint2, arm_joint3, arm_joint4, arm_joint5].
   */
  void subscriberGoalPoseHandler(const std_msgs::Float64MultiArray &msg);

  /**
   * @brief Handles the feedback message of the interactive marker.
   * @param msg Contains the 6D pose of the marker given in 7D with the 3D pose and 4D quaternion.
   */
  void interactiveMarkerHandler(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg);

  /**
   * @brief Loops through the loaded octree and creates an occupancy map if any node is occupied between minZ and maxZ.
   * Also adds all neighbours to the AStarNodes map.
   */
  void create2DMap();

  /**
   * @brief Loops through the occupancy map and inflates it by a radius given by obstacleInflationRadius.
   */
  void inflate2DMap();

  /**
   * @brief Finds a combined 2D and 8D trajectory from the current robot pose
   * such that it reaches a requested end effector pose, saves it as an 8D trajectory to poses, and publishes the trajectory.
   * @param poseEndEffector The 6D pose of the end effector to which the robot should plan, given in [x, y, z, R, P, Y].
   * @return False if no trajectory could be found or if no free pose could be found to the end effector, returns true otherwise.
   */
  bool findTrajectoryEndEffector(const std::vector<Real> &poseEndEffector);

  /**
   * @brief Finds a combined 2D and 8D trajectory from the current robot pose
   * such that it reaches a requested goal pose, saves it to poses, and publishes the trajectory.
   * @param poseGoalNew The 8D goal pose to which the robot should plan.
   * @return False if no trajectory could be found and true otherwise.
   */
  bool findTrajectoryPose(const std::vector<Real> &poseGoalNew);

  /**
   * @brief Uses inverse kinematic to find a possible 8D pose for a requested 6D end effector pose.
   * @param poseEndEffector The 6D pose, given in [x, y, z, R, P, Y], for which a free pose should be found.
   * @return Returns true if a free pose could be found and false otherwise.
   */
  bool findGoalPose(const std::vector<Real> &poseEndEffector);

  /**
   * @brief Finds a 2D and 8D trajectory to the goal pose saved in poseGoal,
   * it is called by both findTrajectoryPose and findTrajectoryEndEffector which both save a new goal pose to poseGoal.
   * @return True if a free trajectory could be found, returns false if either the 2D or the 8D search fails.
   */
  bool findTrajectory();

  /**
   * @brief Finds a 2D path from the cell of the current robot position (poseCurrent) to the cell of poseGoal,
   * then shortens the path by distanceBirrtStarPlanning, smoothens the path and saves it as 8D poses using poseInit for the arm joints.
   * @return True of a free path could be found, false otherwise.
   */
  bool findTrajectoryRetractedArm();

  /**
   * @brief Finds an 8D path using birrt star from the last item in poses to poseGoal and also adds the remaining path to poses.
   * @return True if a path could be found, false if the planner could not be initialized or no plan could be found.
   */
  bool findTrajectoryBirrtStar();

  /**
   * @brief Finds an A* actual path between two points and saves it to AStarPath.
   * If no path could be found, the flag valid is set to false and true otherwise.
   * @param startPoint The start point for the path planning in metric coordinates.
   * @param goalPoint The goal point for the path planning in metric coordinates.
   */
  void findAStarPath(const Tuple2D startPoint, const Tuple2D goalPoint);

  /**
   * @brief Expands an AStarNode by its neighbours, if they are free and adds them to openList.
   * @param node The AStarNode that is to be expanded by its neighbours.
   */
  void expandAStarNode(const AStarNode &node);

  /**
   * @brief Inserts a node to openList and sorts it using an integer based binary tree according to its F cost value.
   * @param node AStarNode to be inserted into the open list.
   */
  void openListInsertNode(AStarNode &node);

  /**
   * @brief Updates a node in openList and sorts it using an integer based binary tree according to its F cost value.
   * @param node AStarNode to be updated within the open list.
   */
  void openListUpdateNode(AStarNode &node);

  /**
   * @brief Removes the front item of openList and resorts it if necessary.
   */
  void openListRemoveFrontNode();

  /**
   * @brief Goes over AStarNodes starting at the goal node, finds the actual cells belonging to the path, and saves it to AStarPath
   */
  void constructAStarPath();

  /**
   * @brief Loops through AStarPath, finds shortest connected line segments, smoothens corners between segments, and saves the trajectory to poses.
   */
  void getSmoothTrajFromAStarPath();

  /**
   * @brief Moves directly between two points and checks if all cells on the connecting line are free according to occupancyMap.
   * @param pointStart The starting point in metric coordinates of the line segment.
   * @param pointEnd The end point in metric coordinates of the line segment.
   * @return False if any occpuied call is found, true otherwise.
   */
  bool isConnectionLineFree(const Tuple2D &pointStart, const Tuple2D &pointEnd);


  // ******************** INLINES ********************


  /**
   * @brief Loads a parameter from the ROS parameter server and saves it to a member varible .
   * If the parameter could not be found or has an invalid name the default value is used instead.
   * @param name The name of the paramter to be loaded by the global node handle nh.
   * @param member The member variable to which the parameter should be saved.
   * @param defaultValue The value which is used saved to member in case the parameter could not be loaded.
   */
  void loadParameter(const string &name, Real &member, const Real &defaultValue);

  /**
   * @brief Checks if an octree node is occupied in the currently loaded octree.
   * @param key The octomap::OcTreeKey to the octree node to be checked.
   * @return Returns true if the node saved in key is occupied, false if the node is occupied or NULL.
   */
  bool isOctreeNodeOccupied(const octomap::OcTreeKey &key) const;

  /**
   * @brief Converts a cell to metric coordinates according to minX, minY and the resolution of occupancyMap (or octree, which is the same)
   * @param cell The cell which is to converted into a metric point.
   * @return Retuns a Tuple2D which contains the metric coordinates of the cell.
   */
  Tuple2D getPointFromCell(const Cell2D &cell) const;

  /**
   * @brief Converts a metric point to an integer cell according to minX, minY and the resolution of occupancyMap (or octree, which is the same)
   * @param point The point which is to converted into an integer cell.
   * @return Retuns a Cell2D which contains the integer coordinates of the point.
   */
  Cell2D getCellFromPoint(const Tuple2D &point) const;

  /**
   * @brief Finds the distance between two cells in cell coordinates.
   * The distance is not metric, but according to the rule that the distance between two adjecent cells is 1.
   * @param cell1 The first cell for finding the distance.
   * @param cell2 The second cell for finding the distance.
   * @return Returns the distance between two cells.
   */
  Real distanceCells(const Cell2D &cell1, const Cell2D &cell2);

  /**
   * @brief Checks if a cell is occpuied according to occupancyMap.
   * @param cell The cell to be checked for occupancy.
   * @return True if the corresponding cell in occupancyMap is true, false otherwise.
   */
  bool isCellOccupied(const Cell2D &cell) const;

  /**
   * @brief Updates the F cost value of an AStarNode accoring to its G cost and an optimal admissable heuristics for grid maps.
   * @param node AStarNode for which the F cost value is to be updated.
   */
  void updateFCost(AStarNode &node) const;
};

} //namespace SquirrelMotionPlanner

#endif /* SQUIRREL_8DOF_PLANNER_PLANNER_H_ */
