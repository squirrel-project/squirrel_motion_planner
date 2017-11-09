#include <squirrel_8dof_planner/squirrel_8dof_planner.h>

namespace SquirrelMotionPlanner
{

// ********************************************************
// ******************** PUBLIC MEMBERS ********************
// ********************************************************

Planner::Planner() :
    nhPrivate("~"), birrtStarPlanner("robotino_robot"), interactiveMarkerServer("endeffector_goal"), octree(NULL)
{
  initializeParameters();
  if (posesFolding.size() == 0)
  {
    ros::shutdown();
    return;
  }

  initializeMessageHandling();

  waitAndSpin(3.0);

  initializeInteractiveMarker();
}

// *********************************************************
// ******************** PRIVATE MEMBERS ********************
// *********************************************************

void Planner::initializeParameters()
{
  poseCurrent.resize(8, 0.0);
  poseGoal.resize(8, 0.0);
  poseInteractiveMarker.resize(6, 0.0);
  birrtStarPlanningNumber = 0;

  loadParameter("normalized_pose_distances", normalizedPoseDistances, Pose());
  if (normalizedPoseDistances.size() != 8)
    ROS_ERROR("Parameter 'normalized_pose_distances' is not of size 8. Trajectories will not be normalized.");

  Pose vectorTmp;
  loadParameter("trajectory_folding_arm", vectorTmp, Pose());
  UInt counter = 0;
  std::vector<Pose> posesFoldingKeyFrames;
  if (vectorTmp.size() % 5 == 0)
  {
    posesFoldingKeyFrames.resize(vectorTmp.size() / 5, Pose(5));
    for (UInt i = 0; i < posesFoldingKeyFrames.size(); ++i)
      for (UInt j = 0; j < 5; ++j)
      {
        posesFoldingKeyFrames[i][j] = vectorTmp[counter];
        ++counter;
      }

    std::vector<Real> normalizedPoseDistancesArm(5);
    normalizedPoseDistancesArm[0] = normalizedPoseDistances[3];
    normalizedPoseDistancesArm[1] = normalizedPoseDistances[4];
    normalizedPoseDistancesArm[2] = normalizedPoseDistances[5];
    normalizedPoseDistancesArm[3] = normalizedPoseDistances[6];
    normalizedPoseDistancesArm[4] = normalizedPoseDistances[7];

    normalizeTrajectory(posesFoldingKeyFrames, posesFolding, normalizedPoseDistancesArm);
  }
  else
    ROS_ERROR("Parameter list 'trajectory_folding_arm' is not divisible by 5. Folding arm trajectory has not been loaded.");




  loadParameter("time_between_poses", timeBetweenPoses, 1.0);
  loadParameter("occupancy_height_min", mapMinZ, 0.0);
  loadParameter("occupancy_height_max", mapMaxZ, 2.0);
  loadParameter("floor_collision_distance", floorCollisionDistance, 3.0);
  loadParameter("astar_safety_distance", obstacleInflationRadius, 0.3);
  loadParameter("astar_smoothing_factor", AStarPathSmoothingFactor, 2.0);
  loadParameter("astar_smoothing_distance", AStarPathSmoothingDistance, 0.2);
  loadParameter("astar_final_smoothed_point_distance", AStarPathSmoothedPointDistance, 0.02);
}

void Planner::initializeMessageHandling()
{
  subscriberPose = nh.subscribe("/arm_controller/joint_states", 1, &Planner::subscriberPoseHandler, this);

  serviceServerGoalMarker = nh.advertiseService("find_interactive_marker_plan", &Planner::serviceCallbackGoalMarker, this);
  serviceServerSendControlCommand = nh.advertiseService("send_trajectory_controller", &Planner::serviceCallbackSendControlCommand, this);
  serviceServerFoldArm = nh.advertiseService("fold_arm", &Planner::serviceCallbackFoldArm, this);
  serviceServerUnfoldArm = nh.advertiseService("unfold_arm", &Planner::serviceCallbackUnfoldArm, this);
  serviceServerGoalPose = nh.advertiseService("find_plan_pose", &Planner::serviceCallbackGoalPose, this);
  serviceServerGoalEndEffector = nh.advertiseService("find_plan_end_effector", &Planner::serviceCallbackGoalEndEffector, this);

  serviceClientOctomap = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");

  publisherOctomap = nh.advertise<octomap_msgs::Octomap>("octomap_planning", 1);
  publisherOccupancyMap = nhPrivate.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1);
  publisher2DPath = nhPrivate.advertise<nav_msgs::Path>("path_2d", 10);
  publisherTrajectoryVisualizer = nhPrivate.advertise<std_msgs::Float64MultiArray>("robot_trajectory_multi_array", 10);
  publisherGoalPose = nhPrivate.advertise<std_msgs::Float64MultiArray>("robot_goal_pose", 10);
  publisherTrajectoryController = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/joint_trajectory_controller/command", 10);
}

void Planner::initializeInteractiveMarker()
{
  interactiveMarker.header.frame_id = "hand_wrist_link";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "endeffector_goal_marker";

  visualization_msgs::InteractiveMarkerControl markerControl;
  markerControl.always_visible = true;

  visualization_msgs::Marker endeffectorMarker;
  endeffectorMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  endeffectorMarker.color.r = endeffectorMarker.color.b = 0.0;
  endeffectorMarker.color.g = 0.8;
  endeffectorMarker.color.a = 0.5;
  endeffectorMarker.pose.position.x = 0.0;
  endeffectorMarker.pose.position.y = 0.0;
  endeffectorMarker.pose.position.z = 0.0;
  endeffectorMarker.pose.orientation.x = 0.0;
  endeffectorMarker.pose.orientation.y = 0.0;
  endeffectorMarker.pose.orientation.z = 0.0;
  endeffectorMarker.pose.orientation.w = 1.0;
  endeffectorMarker.scale.x = 1.0;
  endeffectorMarker.scale.y = 1.0;
  endeffectorMarker.scale.z = 1.0;
  endeffectorMarker.mesh_resource = "package://squirrel_8dof_planner/config/squirrel-hand.dae";
  endeffectorMarker.mesh_use_embedded_materials = false;
  markerControl.markers.push_back(endeffectorMarker);
  interactiveMarker.controls.push_back(markerControl);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  interactiveMarkerServer.insert(interactiveMarker, boost::bind(&Planner::interactiveMarkerHandler, this, _1));
  interactiveMarkerServer.applyChanges();
}

void Planner::publishOccupancyMap() const
{
  if (publisherOccupancyMap.getNumSubscribers() == 0)
    return;

  nav_msgs::OccupancyGrid msg;

  msg.info.width = occupancyMap.size();
  msg.info.height = occupancyMap[0].size();
  msg.info.resolution = mapResolution;
  msg.info.origin.position.x = mapMinX;
  msg.info.origin.position.y = mapMinY;
  msg.info.origin.position.z = 0.0;
  msg.info.map_load_time = ros::Time::now();
  msg.header.seq = 0;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.data.resize(msg.info.width * msg.info.height);
  UInt counter = 0;
  for (UInt j = 0; j < occupancyMap[0].size(); ++j)
  {
    for (UInt i = 0; i < occupancyMap.size(); ++i)
    {
      msg.data[counter] = occupancyMap[i][j] == true ? 100 : 0;
      ++counter;
    }
  }

  publisherOccupancyMap.publish(msg);
}

void Planner::publish2DPath() const
{
  if (publisher2DPath.getNumSubscribers() == 0)
    return;

  nav_msgs::Path msg;

  msg.header.frame_id = "map";
  msg.header.seq = 0;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.pose.position.z = 0.01;
  for (int i = 0; i < posesTrajectory.size(); ++i)
  {
    poseStamped.pose.position.x = posesTrajectory[i][0];
    poseStamped.pose.position.y = posesTrajectory[i][1];
    msg.poses.push_back(poseStamped);
  }

  publisher2DPath.publish(msg);
}

void Planner::publishTrajectoryVisualizer() const
{
  if (publisherTrajectoryVisualizer.getNumSubscribers() == 0 || posesTrajectoryNormalized.size() <= 1)
    return;

  std_msgs::Float64MultiArray msg;

  msg.layout.data_offset = 0;
  msg.layout.dim.resize(2);
  msg.layout.dim[0].label = "pose";
  msg.layout.dim[0].size = posesTrajectoryNormalized.size();
  msg.layout.dim[0].stride = posesTrajectoryNormalized.size() * 8;
  msg.layout.dim[1].label = "joint";
  msg.layout.dim[1].size = 8;
  msg.layout.dim[1].stride = 8;

  msg.data.resize(posesTrajectoryNormalized.size() * 8);

  UInt index = 0;
  for (UInt i = 0; i < posesTrajectoryNormalized.size(); ++i)
  {
    for (UInt j = 0; j < 8; ++j)
    {
      msg.data[index] = posesTrajectoryNormalized[i][j];
      ++index;
    }
  }

  publisherTrajectoryVisualizer.publish(msg);
}

void Planner::publishGoalPose() const
{
  if (publisherGoalPose.getNumSubscribers() == 0 || poseGoal.size() != 8)
    return;

  std_msgs::Float64MultiArray msg;

  msg.layout.data_offset = 0;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "joint";
  msg.layout.dim[0].size = 8;
  msg.layout.dim[0].stride = 8;

  msg.data.resize(8);

  for (UInt i = 0; i < 8; ++i)
    msg.data[i] = poseGoal[i];

  publisherGoalPose.publish(msg);
}

void Planner::publishTrajectoryController()
{
  if (publisherTrajectoryController.getNumSubscribers() == 0 || posesTrajectoryNormalized.size() < 1)
    return;

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names.resize(8);
  msg.joint_names[0] = "base_jointx";
  msg.joint_names[1] = "base_jointy";
  msg.joint_names[2] = "base_jointz";
  msg.joint_names[3] = "arm_joint1";
  msg.joint_names[4] = "arm_joint2";
  msg.joint_names[5] = "arm_joint3";
  msg.joint_names[6] = "arm_joint4";
  msg.joint_names[7] = "arm_joint5";
  msg.points.resize(posesTrajectoryNormalized.size());

  ros::Duration time(0.0);
  for (UInt i = 0; i < posesTrajectoryNormalized.size(); ++i)
  {
    time += ros::Duration(timeBetweenPoses);
    msg.points[i].positions = posesTrajectoryNormalized[i];
    msg.points[i].time_from_start = time;
  }

  publisherTrajectoryController.publish(msg);
}

void Planner::subscriberPoseHandler(const sensor_msgs::JointState &msg)
{
  for (UInt i = 0; i < msg.position.size(); ++i)
  {
    if (msg.name[i] == "arm_joint1")
      poseCurrent[3] = msg.position[i];
    else if (msg.name[i] == "arm_joint2")
      poseCurrent[4] = msg.position[i];
    else if (msg.name[i] == "arm_joint3")
      poseCurrent[5] = msg.position[i];
    else if (msg.name[i] == "arm_joint4")
      poseCurrent[6] = msg.position[i];
    else if (msg.name[i] == "arm_joint5")
      poseCurrent[7] = msg.position[i];
    else if (msg.name[i] == "base_jointx")
      poseCurrent[0] = msg.position[i];
    else if (msg.name[i] == "base_jointy")
      poseCurrent[1] = msg.position[i];
    else if (msg.name[i] == "base_jointz")
      poseCurrent[2] = msg.position[i];
  }
}

bool Planner::serviceCallbackGoalPose(squirrel_motion_planner_msgs::PlanPoseRequest &req, squirrel_motion_planner_msgs::PlanPoseResponse &res)
{
  checkSelfCollision = req.check_self_collision;
  checkMapCollision = req.check_octomap_collision;

  if (req.joints.size() != 8)
  {
    res.result = squirrel_motion_planner_msgs::PlanPoseResponse::WRONG_JOINT_ARRAY_DIM;
    return true;
  }

  if (checkMapCollision && !serviceCallGetOctomap())
  {
    res.result = squirrel_motion_planner_msgs::PlanPoseResponse::NO_OCTOMAP_AVAILABLE;
    return true;
  }

  poseGoal = req.joints;
  posesTrajectory.clear();

  if (req.fold_arm && Tuple2D(poseGoal[0], poseGoal[1]).distance(Tuple2D(poseCurrent[0], poseCurrent[1])) > req.min_distance_before_folding)
  {
    ROS_ERROR("COMBINED 2DOF AND 8DOF PLANNING NOT IMPLEMENTED!");
    return false;

    createOccupancyMapFromOctomap();
    inflateOccupancyMap();
    createAStarNodesMap();

    if (!findTrajectory2D())
    {
      res.result = squirrel_motion_planner_msgs::PlanPoseResponse::ERROR_2DOF_PLANNING;
      return true;
    }

    if (!findTrajectory8D(posesTrajectory.back(), poseGoal, req.max_planning_time))
    {
      res.result = squirrel_motion_planner_msgs::PlanPoseResponse::ERROR_8DOF_PLANNING;
      return true;
    }
  }
  else
  {
    if (!findTrajectory8D(poseCurrent, poseGoal, req.max_planning_time))
    {
      res.result = squirrel_motion_planner_msgs::PlanPoseResponse::ERROR_8DOF_PLANNING;
      return true;
    }

    normalizeTrajectory(posesTrajectory, posesTrajectoryNormalized, normalizedPoseDistances);
  }

  res.result = squirrel_motion_planner_msgs::PlanPoseResponse::SUCCESS;
  publish2DPath();
  publishTrajectoryVisualizer();
  return true;
}

bool Planner::serviceCallbackGoalEndEffector(squirrel_motion_planner_msgs::PlanEndEffectorRequest &req,
                                             squirrel_motion_planner_msgs::PlanEndEffectorResponse &res)
{
  checkSelfCollision = req.check_self_collision;
  checkMapCollision = req.check_octomap_collision;

  if (req.positions.size() != 6)
  {
    res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::WRONG_ARRAY_DIMENSION;
    return true;
  }

  if (checkMapCollision && !serviceCallGetOctomap())
  {
    res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::NO_OCTOMAP_AVAILABLE;
    return true;
  }

  int goalPoseResult = findGoalPose(req.positions);
  if (goalPoseResult == 1)
  {
    res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::COLLISION_GOAL_POSE;
    return true;
  }
  else if (goalPoseResult == 2)
  {
    res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::INVALID_END_EFFECTOR_POSE;
    return true;
  }

  posesTrajectory.clear();
  if (req.fold_arm && Tuple2D(poseGoal[0], poseGoal[1]).distance(Tuple2D(poseCurrent[0], poseCurrent[1])) > req.min_distance_before_folding)
  {
    ROS_ERROR("COMBINED 2DOF AND 8DOF PLANNING NOT IMPLEMENTED!");
    return false;

    createOccupancyMapFromOctomap();
    inflateOccupancyMap();
    createAStarNodesMap();

    if (!findTrajectory2D())
    {
      res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::ERROR_2DOF_PLANNING;
      return true;
    }

    if (!findTrajectory8D(posesTrajectory.back(), poseGoal, req.max_planning_time))
    {
      res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::ERROR_8DOF_PLANNING;
      return true;
    }
  }
  else
  {
    if (!findTrajectory8D(poseCurrent, poseGoal, req.max_planning_time))
    {
      res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::ERROR_8DOF_PLANNING;
      return true;
    }

    normalizeTrajectory(posesTrajectory, posesTrajectoryNormalized, normalizedPoseDistances);
  }

  res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::SUCCESS;
  publish2DPath();
  publishTrajectoryVisualizer();
  return true;
}

bool Planner::serviceCallbackGoalMarker(squirrel_motion_planner_msgs::PlanEndEffectorRequest &req, squirrel_motion_planner_msgs::PlanEndEffectorResponse &res)
{
  checkSelfCollision = req.check_self_collision;
  checkMapCollision = req.check_octomap_collision;

  if (checkMapCollision && !serviceCallGetOctomap())
  {
    res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::NO_OCTOMAP_AVAILABLE;
    return true;
  }

  int goalPoseResult = findGoalPose(poseInteractiveMarker);
  if (goalPoseResult == 1)
  {
    res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::COLLISION_GOAL_POSE;
    return true;
  }
  else if (goalPoseResult == 2)
  {
    res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::INVALID_END_EFFECTOR_POSE;
    return true;
  }

  posesTrajectory.clear();
  if (req.fold_arm && Tuple2D(poseGoal[0], poseGoal[1]).distance(Tuple2D(poseCurrent[0], poseCurrent[1])) > req.min_distance_before_folding)
  {
    ROS_ERROR("COMBINED 2DOF AND 8DOF PLANNING NOT IMPLEMENTED!");
    return false;

    createOccupancyMapFromOctomap();
    inflateOccupancyMap();
    createAStarNodesMap();

    if (!findTrajectory2D())
    {
      res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::ERROR_2DOF_PLANNING;
      return true;
    }

    if (!findTrajectory8D(posesTrajectory.back(), poseGoal, req.max_planning_time))
    {
      res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::ERROR_8DOF_PLANNING;
      return true;
    }
  }
  else
  {
    if (!findTrajectory8D(poseCurrent, poseGoal, req.max_planning_time))
    {
      res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::ERROR_8DOF_PLANNING;
      return true;
    }

    normalizeTrajectory(posesTrajectory, posesTrajectoryNormalized, normalizedPoseDistances);
  }

  res.result = squirrel_motion_planner_msgs::PlanEndEffectorResponse::SUCCESS;
  publish2DPath();
  publishTrajectoryVisualizer();
  return true;
}

bool Planner::serviceCallbackSendControlCommand(squirrel_motion_planner_msgs::SendControlCommandRequest &req,
                                                squirrel_motion_planner_msgs::SendControlCommandResponse &res)
{
  if (posesTrajectory.size() < 2)
  {
    res.result = squirrel_motion_planner_msgs::SendControlCommandResponse::NO_TRAJECTORY_AVAILABLE;
    return true;
  }

  if (!isRobotAtTrajectoryStart())
  {
    res.result = squirrel_motion_planner_msgs::SendControlCommandResponse::ROBOT_MOVED_AFTER_PLANNING;
    return true;
  }

  res.result = squirrel_motion_planner_msgs::SendControlCommandResponse::SUCCESS;
  publishTrajectoryController();
  return true;
}

bool Planner::serviceCallbackFoldArm(squirrel_motion_planner_msgs::FoldArmRequest &req, squirrel_motion_planner_msgs::FoldArmResponse &res)
{
  checkSelfCollision = req.check_self_collision;
  checkMapCollision = req.check_octomap_collision;

  if (isArmFolded())
  {
    res.result = squirrel_motion_planner_msgs::FoldArmResponse::ARM_ALREADY_FOLDED;
    return true;
  }

  posesTrajectoryNormalized.clear();

  if (!isArmStretched())
  {
    if (checkMapCollision && !serviceCallGetOctomap())
    {
      res.result = squirrel_motion_planner_msgs::FoldArmResponse::NO_OCTOMAP_AVAILABLE;
      return true;
    }

    Pose poseTmp = poseCurrent;
    copyArmToRobotPose(posesFolding.back(), poseTmp);
    posesTrajectory.clear();
    if (!findTrajectory8D(poseCurrent, poseTmp, 2.0))
    {
      res.result = squirrel_motion_planner_msgs::FoldArmResponse::ERROR_8DOF_PLANNING;
      return true;
    }
    normalizeTrajectory(posesTrajectory, posesTrajectoryNormalized, normalizedPoseDistances);

    for (Trajectory::reverse_iterator it = posesFolding.rbegin() + 1; it != posesFolding.rend(); ++it)
    {
      copyArmToRobotPose(*it, poseTmp);
      if (!birrtStarPlanner.isConfigValid(poseTmp, checkSelfCollision, checkMapCollision))
      {
        res.result = squirrel_motion_planner_msgs::FoldArmResponse::COLLISION_FOLDING;
        return true;
      }
      posesTrajectoryNormalized.push_back(poseTmp);
    }
  }
  else
  {
    Pose poseTmp = poseCurrent;
    for (Trajectory::reverse_iterator it = posesFolding.rbegin(); it != posesFolding.rend(); ++it)
    {
      copyArmToRobotPose(*it, poseTmp);
      if (!birrtStarPlanner.isConfigValid(poseTmp, checkSelfCollision, checkMapCollision))
      {
        res.result = squirrel_motion_planner_msgs::FoldArmResponse::COLLISION_FOLDING;
        return true;
      }
      posesTrajectoryNormalized.push_back(poseTmp);
    }
  }

  res.result = squirrel_motion_planner_msgs::FoldArmResponse::SUCCESS;

  publishTrajectoryController();
  return true;
}

bool Planner::serviceCallbackUnfoldArm(squirrel_motion_planner_msgs::UnfoldArmRequest &req, squirrel_motion_planner_msgs::UnfoldArmResponse &res)
{
  checkSelfCollision = req.check_self_collision;
  checkMapCollision = req.check_octomap_collision;

  if (!isArmFolded())
  {
    res.result = squirrel_motion_planner_msgs::UnfoldArmResponse::ARM_NOT_FOLDED;
    return true;
  }
  if (checkMapCollision && !serviceCallGetOctomap())
  {
    res.result = squirrel_motion_planner_msgs::UnfoldArmResponse::NO_OCTOMAP_AVAILABLE;
    return true;
  }

  posesTrajectoryNormalized.clear();
  Pose poseTmp = poseCurrent;

  for (Trajectory::iterator it = posesFolding.begin(); it != posesFolding.end(); ++it)
  {
    copyArmToRobotPose(*it, poseTmp);
    if (!birrtStarPlanner.isConfigValid(poseTmp, checkSelfCollision, checkMapCollision))
    {
      res.result = squirrel_motion_planner_msgs::UnfoldArmResponse::COLLISION_UNFOLDING;
      return true;
    }
    posesTrajectoryNormalized.push_back(poseTmp);
  }

  res.result = squirrel_motion_planner_msgs::UnfoldArmResponse::SUCCESS;

  publishTrajectoryController();
  return true;
}

bool Planner::serviceCallGetOctomap()
{
  octomap_msgs::GetOctomapRequest req;
  octomap_msgs::GetOctomapResponse res;
  if (!serviceClientOctomap.call(req, res))
  {
    ROS_ERROR("Could not receive a new octomap from 'octomap_server_node', planning will not be executed.");
    return false;
  }

  if (octree)
    delete octree;

  octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(res.map));
  if (octree == NULL)
  {
    ROS_ERROR("Received octomap is empty, planning will not be executed.");
    return false;
  }

  Real dummy;
  octree->getMetricMin(mapMinX, mapMinY, dummy);
  octree->getMetricMax(mapMaxX, mapMaxY, dummy);

  octomap::OcTreeKey keyTmp = octree->coordToKey(poseCurrent[0], poseCurrent[1], -octree->getResolution() * 0.5);

  UInt xLimitMin = -(Int)(floorCollisionDistance / octree->getResolution()) + (Int)keyTmp[0];
  UInt xLimitMax = (Int)(floorCollisionDistance / octree->getResolution()) + (Int)keyTmp[0];
  UInt yLimitMin = -(Int)(floorCollisionDistance / octree->getResolution()) + (Int)keyTmp[1];
  UInt yLimitMax = (Int)(floorCollisionDistance / octree->getResolution()) + (Int)keyTmp[1];

  for (UInt x = xLimitMin; x <= xLimitMax; ++x)
    for (UInt y = yLimitMin; y <= yLimitMax; ++y)
    {
      keyTmp[0] = x;
      keyTmp[1] = y;
      octree->updateNode(keyTmp, true, true);
    }

  octree->prune();

  if (publisherOctomap.getNumSubscribers() > 0)
  {
    octomap_msgs::Octomap msgOctomap;
    octomap_msgs::fullMapToMsg(*octree, msgOctomap);
    msgOctomap.header.frame_id = "map";
    msgOctomap.header.stamp = ros::Time::now();
    publisherOctomap.publish(msgOctomap);
  }

  birrtStarPlanner.setOctree(octree);
  return true;
}

void Planner::interactiveMarkerHandler(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg)
{
  poseInteractiveMarker[0] = msg->pose.position.x;
  poseInteractiveMarker[1] = msg->pose.position.y;
  poseInteractiveMarker[2] = msg->pose.position.z;
  tf::Matrix3x3 mat(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
  double r, p, y;
  mat.getRPY(r, p, y);
  poseInteractiveMarker[3] = r;
  poseInteractiveMarker[4] = p;
  poseInteractiveMarker[5] = y;
}

void Planner::createOccupancyMapFromOctomap()
{
  octomap::OcTree* octree;
  Real octreeMinZ, octreeMaxZ;
  Int octreeKeyMinX, octreeKeyMinY, octreeKeyMinZ;
  Int octreeKeyMaxX, octreeKeyMaxY, octreeKeyMaxZ;

  octree->getMetricMin(mapMinX, mapMinY, octreeMinZ);
  octree->getMetricMax(mapMaxX, mapMaxY, octreeMaxZ);
  mapResolution = octree->getResolution();
  mapResolutionRecip = 1.0 / mapResolution;

  octomap::OcTreeKey keyMinPlane = octree->coordToKey(mapMinX + mapResolution * 0.5, mapMinY + mapResolution * 0.5, octreeMinZ + mapResolution * 0.5);
  octomap::OcTreeKey keyMaxPlane = octree->coordToKey(mapMaxX - mapResolution * 0.5, mapMaxY - mapResolution * 0.5, octreeMaxZ - mapResolution * 0.5);

  octreeKeyMinX = keyMinPlane[0];
  octreeKeyMinY = keyMinPlane[1];
  octreeKeyMaxX = keyMaxPlane[0];
  octreeKeyMaxY = keyMaxPlane[1];
  octreeKeyMinZ = octree->coordToKey(0.0, 0.0, mapMinZ + mapResolution * 0.5)[2];
  octreeKeyMaxZ = octree->coordToKey(0.0, 0.0, mapMaxZ - mapResolution * 0.5)[2];

  occupancyMap.clear();
  occupancyMap.resize(octreeKeyMaxX - octreeKeyMinX + 1, std::vector<bool>(octreeKeyMaxY - octreeKeyMinY + 1, false));

  octomap::OcTreeKey key;
  for (UInt x = 0; x < occupancyMap.size(); ++x)
  {
    for (UInt y = 0; y < occupancyMap[0].size(); ++y)
    {
      for (UInt z = 0; z < octreeKeyMaxZ - octreeKeyMinZ; ++z)
      {
        key[0] = x + octreeKeyMinX;
        key[1] = y + octreeKeyMinY;
        key[2] = z + octreeKeyMinZ;
        if (isOctreeNodeOccupied(key))
        {
          occupancyMap[x][y] = true;
          z = octreeKeyMaxZ - octreeKeyMinZ;
        }
      }
    }
  }
}

void Planner::inflateOccupancyMap()
{
  std::vector<std::vector<Real> > inflationMap(occupancyMap.size(), std::vector<Real>(occupancyMap[0].size(), -1.0));
  std::deque<Cell2D> queue;

  for (UInt x = 0; x < occupancyMap.size(); ++x)
    for (UInt y = 0; y < occupancyMap[0].size(); ++y)
      if (occupancyMap[x][y])
      {
        queue.push_back(Cell2D(x, y));
        inflationMap[x][y] = 0.0;
      }

  while (queue.size() > 0)
  {
    const Cell2D cellCenter = queue[0];
    queue.pop_front();

    Cell2D cell(cellCenter.x - 1, cellCenter.y - 1);

    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < inflationMap.size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y + 1;
    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x + 1;
    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < inflationMap.size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y - 1;
    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }
  }

  for (UInt x = 0; x < inflationMap.size(); ++x)
    for (UInt y = 0; y < inflationMap[0].size(); ++y)
      if (inflationMap[x][y] > 0.0)
        occupancyMap[x][y] = true;
}

void Planner::createAStarNodesMap()
{
  AStarNodes.clear();
  AStarNodes.resize(occupancyMap.size(), std::vector<AStarNode>(occupancyMap[0].size()));

  for (UInt x = 0; x < AStarNodes.size(); ++x)
  {
    for (UInt y = 0; y < AStarNodes[0].size(); ++y)
    {
      AStarNodes[x][y].cell = Cell2D(x, y);
      if (occupancyMap[x][y])
      {
        AStarNodes[x][y].occupied = AStarNodes[x][y].closed = true;
        continue;
      }

      if (x + 1 < AStarNodes.size() && y + 1 < AStarNodes[0].size() && !occupancyMap[x + 1][y + 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x + 1][y + 1], M_SQRT2));
      if (x + 1 < AStarNodes.size() && !occupancyMap[x + 1][y])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x + 1][y], 1.0));
      if (x + 1 < AStarNodes.size() && y - 1 < AStarNodes[0].size() && !occupancyMap[x + 1][y - 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x + 1][y - 1], M_SQRT2));
      if (y + 1 < AStarNodes[0].size() && !occupancyMap[x][y + 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x][y + 1], 1.0));
      if (y - 1 < AStarNodes[0].size() && !occupancyMap[x][y - 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x][y - 1], 1.0));
      if (x - 1 < AStarNodes.size() && y + 1 < AStarNodes[0].size() && !occupancyMap[x - 1][y + 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x - 1][y + 1], M_SQRT2));
      if (x - 1 < AStarNodes.size() && !occupancyMap[x - 1][y])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x - 1][y], 1.0));
      if (x - 1 < AStarNodes.size() && y - 1 < AStarNodes[0].size() && !occupancyMap[x - 1][y - 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x - 1][y - 1], M_SQRT2));
    }
  }
}

int Planner::findGoalPose(const Pose &poseEndEffector)
{
  std::vector<std::pair<Real, Real> > endEffectorDeviations(6);
  endEffectorDeviations[0] = std::make_pair<Real, Real>(-0.005, 0.005);
  endEffectorDeviations[1] = std::make_pair<Real, Real>(-0.005, 0.005);
  endEffectorDeviations[2] = std::make_pair<Real, Real>(-0.005, 0.005);
  endEffectorDeviations[3] = std::make_pair<Real, Real>(-0.025, 0.025);
  endEffectorDeviations[4] = std::make_pair<Real, Real>(-0.025, 0.025);
  endEffectorDeviations[5] = std::make_pair<Real, Real>(-0.025, 0.025);

  Real dist;
  bool isDownward = fabs(Tuple3D(0.0, 0.0, 1.0) * getEndEffectorDirection(poseEndEffector)) < 0.9 ? false : true;
  Pose poseInitializer(8);

  // this checks if the hand  direction is approximately 30 deg with the xy-plane
  if (isDownward)
  {
    dist = 0.44;

      poseInitializer[3] = -0.8;
      poseInitializer[4] = 0.8;
      poseInitializer[5] = 0.0;
      poseInitializer[6] = -1.5;
      poseInitializer[7] = 0.0;
  }
  else
  {
    dist = 0.47;

    poseInitializer[3] = -1.2;
    poseInitializer[4] = 1.1;
    poseInitializer[5] = 0.0;
    poseInitializer[6] = 0.7;
    poseInitializer[7] = -1.5;
  }

  Real angle = 0.0;
  bool foundSinglePose = false;
  while (angle < 2 * M_PI)
  {
    poseInitializer[0] = poseEndEffector[0] - dist * cos(angle);
    poseInitializer[1] = poseEndEffector[1] - dist * sin(angle);
    poseInitializer[2] = isDownward ? angle + 0.99 : angle + 0.99;
    if (birrtStarPlanner.getFullPoseFromEEPose(poseEndEffector, endEffectorDeviations, poseInitializer, poseGoal))
    {
      foundSinglePose = true;

      if (birrtStarPlanner.isConfigValid(poseGoal, checkSelfCollision, checkMapCollision))
      {
        publishGoalPose();
        return 0;
      }
    }

    angle += 0.5;
  }

  poseGoal.clear();
  if (foundSinglePose)
    return 1;
  else
    return 2;
}

bool Planner::findTrajectory2D()
{
  findAStarPath(Tuple2D(poseCurrent[0], poseCurrent[1]), Tuple2D(poseGoal[0], poseGoal[1]));
  if (!AStarPath.valid)
    return false;

  Real distanceFromGoal = 0.0;
  while (distanceFromGoal < distance8DofPlanning && AStarPath.cells.size() > 2)
  {
    distanceFromGoal += AStarPath.points.back().distance(AStarPath.points.end()[-2]);
    AStarPath.points.pop_back();
    AStarPath.cells.pop_back();
  }

  getSmoothTrajFromAStarPath();
  return true;
}

bool Planner::findTrajectory8D(const Pose &poseStart, const Pose &poseGoal, const Real maxPlanningTime)
{
  if (birrtStarPlanningNumber > 0)
    birrtStarPlanner.reset_planner_and_config();

  std::vector<Real> dimX(2), dimY(2);
  dimX[0] = mapMinX;
  dimX[1] = mapMaxX;
  dimY[0] = mapMinY;
  dimY[1] = mapMaxY;

  birrtStarPlanner.setPlanningSceneInfo(dimX, dimY, "scenario");

  if (!birrtStarPlanner.init_planner(poseStart, poseGoal, 1, checkSelfCollision, checkMapCollision))
    return false;

  if (!birrtStarPlanner.run_planner(1, 1, maxPlanningTime, false, 0.0, birrtStarPlanningNumber))
    return false;

  const Trajectory &trajectoryBirrtStar = birrtStarPlanner.getJointTrajectoryRef();

  for (UInt i = 0; i < trajectoryBirrtStar.size(); ++i)
    posesTrajectory.push_back(trajectoryBirrtStar[i]);

  ++birrtStarPlanningNumber;
  return true;
}

void Planner::findAStarPath(const Tuple2D startPoint, const Tuple2D goalPoint)
{
  AStarPath.valid = false;
  AStarPath.cells.clear();
  AStarPath.points.clear();

  const UInt sizeX = AStarNodes.size();
  const UInt sizeY = AStarNodes[0].size();
  for (UInt x = 0; x < sizeX; ++x)
    for (UInt y = 0; y < sizeY; ++y)
      if (!AStarNodes[x][y].occupied)
        AStarNodes[x][y].open = AStarNodes[x][y].closed = false;

  AStarCellStart = getCellFromPoint(startPoint);
  AStarCellGoal = getCellFromPoint(goalPoint);

  if (AStarCellStart == AStarCellGoal)
  {
    AStarPath.cells.push_back(AStarCellGoal);
    AStarPath.points.push_back(getPointFromCell(AStarCellGoal));
    AStarPath.valid = true;
    return;
  }

  openListNodes.clear();
  openListNodes.push_back(Cell2D(-1, -1));
  openListNodes.push_back(AStarCellStart);

  //initialize starting nodes
  AStarNodes[AStarCellStart.x][AStarCellStart.y].cellParent = Cell2D(-1, -1);
  AStarNodes[AStarCellStart.x][AStarCellStart.y].g = 0;
  AStarNodes[AStarCellStart.x][AStarCellStart.y].indexOpenList = 1;
  updateFCost(AStarNodes[AStarCellStart.x][AStarCellStart.y]);

  while (openListNodes.size() > 1)
  {
    if (AStarNodes[openListNodes[1].x][openListNodes[1].y].cell == AStarCellGoal)
    {
      constructAStarPath();
      AStarPath.valid = true;
      return;
    }

    openListRemoveFrontNode();
    AStarNodes[openListNodes[1].x][openListNodes[1].y].closed = true;
    expandAStarNode(AStarNodes[openListNodes[1].x][openListNodes[1].y]);
  }

  ROS_WARN("No path found after expanding all nodes.");
}

void Planner::expandAStarNode(const AStarNode &node)
{
  for (UInt i = 0; i < node.neighbors.size(); ++i)
  {
    AStarNode &nodeNew = *node.neighbors[i].first;
    if (!nodeNew.closed)
    {
      if (!nodeNew.open)
      {
        nodeNew.open = true;
        nodeNew.cellParent = node.cell;
        nodeNew.g = node.g + node.neighbors[i].second;
        updateFCost(nodeNew);
        openListInsertNode(nodeNew);
      }
      else if (nodeNew.g > node.g + node.neighbors[i].second)
      {
        nodeNew.cellParent = node.cell;
        nodeNew.g = node.g + node.neighbors[i].second;
        updateFCost(nodeNew);
        openListUpdateNode(nodeNew);
      }
    }
  }
}

void Planner::openListInsertNode(AStarNode &node)
{
  openListNodes.push_back(node.cell);
  UInt index = openListNodes.size() - 1;
  UInt indexHalf = index / 2;
  node.indexOpenList = index;

  while (indexHalf > 0 && AStarNodes[openListNodes[index].x][openListNodes[index].y].f < AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void Planner::openListUpdateNode(AStarNode &node)
{
  UInt index = node.indexOpenList;
  UInt indexHalf = index / 2;

  while (indexHalf > 0 && AStarNodes[openListNodes[index].x][openListNodes[index].y].f < AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void Planner::openListRemoveFrontNode()
{
  openListNodes[1] = openListNodes.back();
  openListNodes.pop_back();
  AStarNodes[openListNodes[1].x][openListNodes[1].y].indexOpenList = 1;

  UInt index = 1, indexDouble = 2, indexDoubleOne = 3;
  while (indexDouble < openListNodes.size())
  {
    if (indexDoubleOne == openListNodes.size())
    {
      if (AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
        break;
    }
    else if (AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f
        || AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
    {
      if (AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f
          < AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDoubleOne];
        openListNodes[indexDoubleOne] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDoubleOne;
        AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].indexOpenList = index;
        index = indexDoubleOne;
      }
    }
    else
      break;
    indexDouble = index * 2;
    indexDoubleOne = indexDouble + 1;
  }
}

void Planner::constructAStarPath()
{
  std::deque<Cell2D> cellList;

  Cell2D cellTemp = AStarCellGoal;
  while (true)
  {
    cellList.push_front(cellTemp);
    if (cellTemp == AStarCellStart)
      break;
    cellTemp = AStarNodes[cellTemp.x][cellTemp.y].cellParent;
  }

  for (std::deque<Cell2D>::const_iterator it = cellList.begin(); it != cellList.end(); ++it)
  {
    AStarPath.cells.push_back(*it);
    AStarPath.points.push_back(getPointFromCell(*it));
  }
}

void Planner::getSmoothTrajFromAStarPath()
{
  std::vector<LineSegment2D> pathSegments;
  std::vector<ParametricFunctionCubic2D> connections;
  UInt indexCurrent = 0;
  Tuple2D startPointCurrent = AStarPath.points[0];

  //find maximal free connecting way points along astar path
  for (UInt i = 1; i < AStarPath.points.size(); ++i)
  {
    if (isConnectionLineFree(startPointCurrent, AStarPath.points[i]))
      continue;

    pathSegments.push_back(LineSegment2D(startPointCurrent, AStarPath.points[i - 1]));
    startPointCurrent = AStarPath.points[i - 1];
    --i;
  }
  pathSegments.push_back(LineSegment2D(startPointCurrent, AStarPath.points.back()));

  if (pathSegments.size() > 1)
  {
    //shorten path segments to lenghts which are kept
    std::vector<Tuple2D> intersectionPoints;
    intersectionPoints.push_back(pathSegments[0].end);
    pathSegments[0].clipEnd(AStarPathSmoothingDistance);
    for (UInt i = 1; i < pathSegments.size() - 1; ++i)
    {
      intersectionPoints.push_back(pathSegments[i].end);
      pathSegments[i].clipBoth(AStarPathSmoothingDistance);
    }
    pathSegments.back().clipStart(AStarPathSmoothingDistance);

    //find smooth parametric cubic function for each intermediate waypoint
    for (UInt i = 0; i < pathSegments.size() - 1; ++i)
      connections.push_back(ParametricFunctionCubic2D(pathSegments[i].end, pathSegments[i + 1].start, intersectionPoints[i], AStarPathSmoothingFactor, 8));
  }

  //find full path length with smoothed corners
  Real pathLengthFull = 0.0;
  for (UInt i = 0; i < connections.size(); ++i)
    pathLengthFull += connections[i].length;
  for (UInt i = 0; i < pathSegments.size(); ++i)
    pathLengthFull += pathSegments[i].length;

  const Real pointDistance = pathLengthFull / round(pathLengthFull / AStarPathSmoothedPointDistance);

  Pose poseTmp(8);
  copyArmToRobotPose(posesFolding.back(), poseTmp);

  //generate points on full smoothed path
  Real distanceOnCurrent = 0.0, distanceTotal = 0.0;
  Tuple2D pointTmp;
  poseTmp[0] = pathSegments[0].start.x;
  poseTmp[1] = pathSegments[0].start.y;
  poseTmp[2] = pathSegments[0].angle;
  posesTrajectory.push_back(poseTmp);
  for (UInt i = 0; i < pathSegments.size() - 1; ++i)
  {
    poseTmp[2] = pathSegments[i].angle;
    while (true)
    {
      distanceOnCurrent += pointDistance;
      if (distanceOnCurrent > pathSegments[i].length)
      {
        distanceOnCurrent = pathSegments[i].length - distanceOnCurrent;
        break;
      }
      pointTmp = pathSegments[i].getPointAbsolute(distanceOnCurrent);
      poseTmp[0] = pointTmp.x;
      poseTmp[1] = pointTmp.y;
      posesTrajectory.push_back(poseTmp);
    }
    while (true)
    {
      distanceOnCurrent += pointDistance;
      if (distanceOnCurrent > connections[i].length)
      {
        distanceOnCurrent = connections[i].length - distanceOnCurrent;
        break;
      }
      pointTmp = connections[i].getPointAbsolute(distanceOnCurrent);
      poseTmp[0] = pointTmp.x;
      poseTmp[1] = pointTmp.y;
      poseTmp[2] = connections[i].getAngleAbsolute(distanceOnCurrent);
      posesTrajectory.push_back(poseTmp);
    }
  }
  poseTmp[2] = pathSegments.back().angle;
  while (true)
  {
    distanceOnCurrent += pointDistance;
    if (distanceOnCurrent > pathSegments.back().length)
      break;
    pointTmp = pathSegments.back().getPointAbsolute(distanceOnCurrent);
    poseTmp[0] = pointTmp.x;
    poseTmp[1] = pointTmp.y;
    posesTrajectory.push_back(poseTmp);
  }
}

bool Planner::isConnectionLineFree(const Tuple2D &pointStart, const Tuple2D &pointEnd)
{
  Tuple2D direction = pointEnd - pointStart;
  Real distanceMax = sqrt(direction.x * direction.x + direction.y * direction.y);
  Real searchFactor = mapResolution * 0.25;
  direction.x = direction.x * searchFactor / distanceMax;
  direction.y = direction.y * searchFactor / distanceMax;

  Tuple2D pointCurrent = pointStart;
  Real distanceCurrent = 0;
  while (distanceCurrent < distanceMax)
  {
    distanceCurrent += searchFactor;
    pointCurrent += direction;
    if (isCellOccupied(getCellFromPoint(pointCurrent)))
      return false;
  }

  return true;
}

void Planner::normalizeTrajectory(const Trajectory &trajectory, Trajectory &trajectoryNormalized, const Pose &normalizedPose)
{
  UInt poseDimension = normalizedPose.size();

  if (poseDimension < 1 || trajectory.size() <= 1 || trajectory[0].size() != poseDimension)
    return;

  UInt poseNextIndex = 1;
  trajectoryNormalized.clear();
  trajectoryNormalized.push_back(trajectory.front());

  while (true)
  {
    const Pose &poseNext = trajectory[poseNextIndex];
    const Pose &poseLast = trajectoryNormalized.back();

    Real frac = fabs(poseNext[0] - poseLast[0]) / normalizedPose[0];
    for (UInt i = 1; i < poseDimension; ++i)
    {
      const Real fracNew = fabs(poseNext[i] - poseLast[i]) / normalizedPose[i];
      if (fracNew > frac)
        frac = fracNew;
    }
    if (frac < 1.0)
    {
      ++poseNextIndex;
      if (poseNextIndex == trajectory.size())
      {
        trajectoryNormalized.push_back(trajectory.back());
        return;
      }
      else
        continue;
    }

    UInt counterMax = (UInt)frac + 1;
    frac = std::ceil(frac);
    const Pose poseLastCopy = trajectoryNormalized.back();
    for (UInt i = 1; i <= counterMax; ++i)
    {
      trajectoryNormalized.push_back(poseLastCopy);
      for (UInt j = 0; j < poseDimension; ++j)
        trajectoryNormalized.back()[j] += i * (poseNext[j] - trajectoryNormalized.back()[j]) / frac;
    }

    ++poseNextIndex;
    if (poseNextIndex == trajectory.size())
      return;
  }
}

Tuple3D Planner::getEndEffectorDirection(const Pose &poseEndEffector)
{
 tf::StampedTransform transform;
 transform.frame_id_ = "zero";
 transform.child_frame_id_ = "pose_goal_end_effector";
 transform.stamp_ = ros::Time::now();
 transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
 tf::Quaternion quaternion;
 quaternion.setRPY(poseEndEffector[3], poseEndEffector[4], poseEndEffector[5]);
 transform.setRotation(quaternion);
 transformer.setTransform(transform);

 tf::Stamped<tf::Vector3> axisOrig, axis;
 axisOrig[0] = 0.0;
 axisOrig[1] = 1.0;
 axisOrig[2] = 0.0;
 axisOrig.frame_id_ = "pose_goal_end_effector";
 axisOrig.stamp_ = ros::Time(0.0);
 axis.stamp_ = ros::Time(0.0);
 transformer.transformVector("zero", axisOrig, axis);

 return Tuple3D(axis[0], axis[1], axis[2]);
}

// ******************** INLINES ********************

inline void Planner::loadParameter(const string &name, Real &member, const Real &defaultValue)
{
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("Can't find ROS parameter '%s'. Using the default value '%.4f' instead.", (nh.getNamespace() + "/" + name).c_str(), defaultValue);
      member = defaultValue;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("ROS parameter '%s' has an invalid format. Using the default value '%.4f' instead.", (nh.getNamespace() + "/" + name).c_str(), defaultValue);
    member = defaultValue;
  }
}

inline void Planner::loadParameter(const string &name, std::vector<Real> &member, const std::vector<Real> &defaultValue)
{
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("Can't find ROS parameter '%s'. Using the default value instead.", (nh.getNamespace() + "/" + name).c_str());
      member = defaultValue;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("ROS parameter '%s' has an invalid format. Using the default value instead.", (nh.getNamespace() + "/" + name).c_str());
    member = defaultValue;
  }
}

inline bool Planner::isOctreeNodeOccupied(const octomap::OcTreeKey &key) const
{
//  octomap::OcTreeNode* node = octree->search(key);
//
//  if (node != NULL)
//    return octree->isNodeOccupied(node);
//  else
//    return false;
  return false;
}

inline Tuple2D Planner::getPointFromCell(const Cell2D &cell) const
{
  return Tuple2D(mapMinX + (cell.x + 0.5) * mapResolution, mapMinY + (cell.y + 0.5) * mapResolution);
}

inline Cell2D Planner::getCellFromPoint(const Tuple2D &point) const
{
  return Cell2D((UInt)((point.x - mapMinX) * mapResolutionRecip), (UInt)((point.y - mapMinY) * mapResolutionRecip));
}

inline Real Planner::distanceCells(const Cell2D &cell1, const Cell2D &cell2)
{
  return sqrt(((Real)cell1.x - (Real)cell2.x) * ((Real)cell1.x - (Real)cell2.x) + ((Real)cell1.y - (Real)cell2.y) * ((Real)cell1.y - (Real)cell2.y));
}

inline bool Planner::isCellOccupied(const Cell2D &cell) const
{
  return occupancyMap[cell.x][cell.y];
}

inline void Planner::updateFCost(AStarNode &node) const
{
  Real dx = abs((Real)node.cell.x - (Real)AStarCellGoal.x);
  Real dy = abs((Real)node.cell.y - (Real)AStarCellGoal.y);
  node.f = node.g + std::min(dx, dy) * M_SQRT2 + abs(dx - dy);
}

inline bool Planner::isArmFolded() const
{
  if (fabs(poseCurrent[3] - posesFolding.front()[0]) > 0.15236 || fabs(poseCurrent[4] - posesFolding.front()[1]) > 0.15236
      || fabs(poseCurrent[5] - posesFolding.front()[2]) > 0.15236 || fabs(poseCurrent[6] - posesFolding.front()[3]) > 0.15236
      || fabs(poseCurrent[7] - posesFolding.front()[4]) > 0.15236)
    return false;

  return true;
}

inline bool Planner::isArmStretched() const
{
  if (fabs(poseCurrent[3] - posesFolding.back()[0]) > 0.05236 || fabs(poseCurrent[4] - posesFolding.back()[1]) > 0.05236
      || fabs(poseCurrent[5] - posesFolding.back()[2]) > 0.05236 || fabs(poseCurrent[6] - posesFolding.back()[3]) > 0.05236
      || fabs(poseCurrent[7] - posesFolding.back()[4]) > 0.05236)
    return false;

  return true;
}

inline bool Planner::isRobotAtTrajectoryStart() const
{
  if (fabs(poseCurrent[0] - posesTrajectoryNormalized.front()[0] > 0.02) || fabs(poseCurrent[1] - posesTrajectoryNormalized.front()[1] > 0.02)
      || fabs(poseCurrent[2] - posesTrajectoryNormalized.front()[2] > 0.05236) || fabs(poseCurrent[3] - posesTrajectoryNormalized.front()[3] > 0.05236)
      || fabs(poseCurrent[4] - posesTrajectoryNormalized.front()[4] > 0.05236) || fabs(poseCurrent[5] - posesTrajectoryNormalized.front()[5] > 0.05236)
      || fabs(poseCurrent[6] - posesTrajectoryNormalized.front()[6] > 0.05236) || fabs(poseCurrent[7] - posesTrajectoryNormalized.front()[7] > 0.05236))
    return false;

  return true;
}

inline void Planner::copyArmToRobotPose(const Pose &poseArm, Pose &poseRobot)
{
  poseRobot[3] = poseArm[0];
  poseRobot[4] = poseArm[1];
  poseRobot[5] = poseArm[2];
  poseRobot[6] = poseArm[3];
  poseRobot[7] = poseArm[4];
}

inline void Planner::waitAndSpin(const Real seconds)
{
  for (UInt i = 0; i < 10 * seconds; ++i)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

} //namespace SquirrelMotionPlanner
