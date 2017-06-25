RViz robot marker package
=========================

This package provides a publisher for publishing robot models in given poses
as `visualization_msgs::MarkerArray` for RViz. The publisher does _not_ use
the TF system at all.

Library usage
=============
1. Include the header file `<rviz_robot_marker/RvizRobotMarkerPublisher.h>`
2. Create an instance of class `rviz_robot_marker::RvizRobotMarkerPublisher(const std::string& markerTopic = "robot_markers_array")`
3. Call the method `const size_t addRobotFromDescription(const std::string& param = "/robot_description")`
   and save the returned ID
4. Call the method `void setRobotPose(const size_t& id, const tf::StampedTransform& baseTransform, const sensor_msgs::JointStateConstPtr& jointStates)`
   to update the robot pose. The base transform should be the transform between
   the fixed frame of RViz (e.g., /map) and the robot's base link frame. The
   publisher will publish the markers in the parent frame of this transform. If
   the transform is empty, the markers will be published in the robot's base
   link (root frame of the robot's TF tree) instead. 
5. Subscribe in RViz to the `visualization_msgs::MarkerArray` topic
   `robot_markers_array`.


Node usage
==========
1. `rosrun rviz_robot_marker publisher_marker --topic /joint_states`
   where `/joint_states` is the topic where you publish the joint state messages
   of type `sensor_msgs::JointState`.
2. Subscribe in RViz to the `visualization_msgs::MarkerArray` topic
   `robot_markers_array`.

The robot markers will be published in the robot's base link frame (root of the
robot's TF tree). You need to publish a TF transform from RViz's fixed frame
to the robot's base link frame yourself.

TODO: Implement a method for changing the header frame of the published markers
for each robot.

