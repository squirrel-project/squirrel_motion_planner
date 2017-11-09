#ifndef INCLUDE_RVIZROBOTMARKERPUBLISHER_H_
#define INCLUDE_RVIZROBOTMARKERPUBLISHER_H_

#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <tf/tf.h>
#include <urdf_model/model.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
namespace rviz_robot_marker {

/**
 * Class for publishing robot models as visualization markers for RViz.
 * This module does *not* use TF.
 */
class RvizRobotMarkerPublisher {
public:
	/**
	 * @brief Constructor.
	 * @param markerTopic Topic name for the robot visualization marker publisher.
	 */
	RvizRobotMarkerPublisher(const std::string& markerTopic = "robot_markers_array");
	/**
	 * @brief Destructor.
	 *
	 * The destructor does *not* send visualization marker deletion messages to RViz.
	 * Call clear() before deleting the instance if you would like to send deletion messages.
	 */
	virtual ~RvizRobotMarkerPublisher();

	/** Mode defining precedence of color values */
	enum ColorMode
	{
		REPLACE_MESH_COLOR,   ///< Use color unless color is explicitly set in URDF definition (replaces mesh color)
		FORCE_COLOR,          ///< Force given color (overrides mesh color and URDF color)
		BLEND                 ///< Blend given color and mesh/URDF color
	};


	/**
	 * @brief Updates an existing robot marker with a new pose.
	 * The robot marker must exist (call addRobotFromDescription() or addRobotWithJointListener() first)
	 *
	 * @param id ID of the robot marker as returned by addRobotFromDescription() or addRobotWithJointListener()
	 * @param baseTransform Transform from a fixed frame (e.g., the map frame) to the base link of the robot.
	 * If the baseTransform.frame_id_ is set, the robot marker will be published in that frame.
	 * If empty, the marker will be published in the base link frame of the robot instead.
	 * @param jointStates New joint angles for the robot's pose.
	 * @throws std::invalid_argument if robot ID is invalid or joint state name vector does not match position vector
	 */
	void setRobotPose(const size_t& id, const tf::StampedTransform& baseTransform, const sensor_msgs::JointStateConstPtr& jointStates);

	/**
	 * @brief Adds a new robot marker from a URDF robot description on the parameter server.
	 * @param param Name of the param from where to load the URDF robot description.
	 * @return Internal ID of the marker for use within this class (this is *not* the ID of the visualization marker).
	 * @throws std::invalid_argument if robot description parameter cannot be found or the URDF model cannot be parsed
	 */
	const size_t addRobotFromDescription(const std::string& param = "/robot_description");

	/**
	 * @brief Adds a new robot marker from a URDF robot description on the parameter server and attaches it to sensor_msgs::JointState topic subscriber.
	 * @param topic The topic name for subscribing to sensor_msgs::JointState messages
	 * @param param Name of the param from where to load the URDF robot description.
	 * @return Internal ID of the marker for use within this class (this is *not* the ID of the visualization marker).
	 * @throws std::invalid_argument if robot description parameter cannot be found or the URDF model cannot be parsed
	 */
	const size_t addRobotWithJointListener(const std::string& topic, const std::string& param = "/robot_description");

	/**
	 * @brief Deletes a robot marker.
	 * This will also send a visualization marker deletion message to RViz.
	 * @param id ID of the robot marker as returned by addRobotFromDescription() or addRobotWithJointListener()
	 * @throws std::invalid_argument if robot ID is invalid
	 */
	void deleteRobot(const size_t& id);


	/**
	 * @brief Deletes all robot markers.
	 * This will also send visualization marker deletion messages to RViz.
	 */
	void clear();

	/**
	 * @brief Sets the color for the robot mesh. This also affects the lighting and transparency of textured meshes.
	 * @param id ID of the robot marker as returned by addRobotFromDescription() or addRobotWithJointListener()
	 * @param color The new color to use for the robot markers.
	 * @param colorMode Color mode defining precedence of colors from mesh, URDF, and color given here
	 * @throws std::invalid_argument if robot ID is invalid
	 */
	void setRobotColor(const size_t& id, const std_msgs::ColorRGBA& color, const ColorMode& colorMode = REPLACE_MESH_COLOR);

	/**
	 * @brief Sets the name of the robot that will be shown in RViz.
	 * @param id ID of the robot marker as returned by addRobotFromDescription() or addRobotWithJointListener()
	 * @param name The new name of the robot.
	 * @throws std::invalid_argument if robot ID is invalid
	 */
	void setRobotName(const size_t& id, const std::string& name);

	/**
	 * @brief Shows the robot again after it has been hidden with hideRobot().
	 * @param id The robot to show.
	 */
	void showRobot(const size_t& id);

	/**
	 * @brief Hides the robot temporarily by sending a deletion message to RViz and not publishing updates anymore.
	 * Call showRobot() to show the robot again.
	 * @param id The robot to hide.
	 */
	void hideRobot(const size_t& id);

protected:
	ros::NodeHandle nh;                              ///< sibling node handle
	ros::Publisher markerPub;                        ///< visualization marker publisher

	/** @brief Internal structure representing a robot */
	struct Robot
	{
		KDL::Tree kdlTree;                                    ///< KDL tree generated from URDF model
		boost::shared_ptr<const urdf::ModelInterface> urdf;   ///< Pointer to the URDF model
		size_t highestMarkerId;                               ///< Highest marker ID used for this robot
		std_msgs::ColorRGBA markerColor;                      ///< Base marker color (also affects textured meshes)
		ColorMode colorMode;                                  ///< Color mode defining color precedence
		std::string markerNS;                                 ///< Marker namespace that will be shown in RViz
		boost::shared_ptr<ros::Subscriber> jointSubscriber;   ///< Joint subscriber added with addRobotWithJointListener(), may be NULL
		bool hide;                                            ///< True if robot should be temporarily hidden, set by showRobot() and hideRobot()
		sensor_msgs::JointStateConstPtr jointStates;          ///< Last known joint states
		tf::StampedTransform baseTransform;                   ///< Last known base transform
	};
	typedef std::map<size_t, boost::shared_ptr<Robot> > Robots;    ///< Type of the robots list
	Robots robots;               ///< list of robots
	size_t highestRobotId;       ///< highest ID assigned so far;

	typedef std::map<std::string, boost::shared_ptr<const urdf::ModelInterface> > UrdfModelMap;  ///< Type of the urdfModels map.
	UrdfModelMap urdfModels;     ///< cache for URDF models

	/**
	 * @brief Generate a KDL model from the URDF model
	 * @param model Pointer to the URDF model
	 * @param robot Pointer the the Robot for filling its kdlTree member
	 * @return true on success
	 */
	bool kdlModelFromURDF(const boost::shared_ptr<const urdf::ModelInterface>& model, boost::shared_ptr<Robot> robot);
	/**
	 * @brief Propagates joint state updates through the KDL tree and stores the new link poses to a given TF buffer.
	 * This is a recursive function.
	 * @param jointStates The new joint state message containing new positions for all joints.
	 * @param segment Current node to update (children will be updated recursively).
	 * @param tf Internal TF transform buffer.
	 */
	bool computeChildTransforms(const sensor_msgs::JointStateConstPtr& jointStates, const KDL::SegmentMap::const_iterator& segment, tf::Transformer& tf) const;

	/**
	 * @brief Callback function for subscriber listening to sensor_msgs::JointState messages.
	 * @param id ID of the robot
	 * @param jointStates Message.
	 */
	void jointStateCallback(const size_t& id, const sensor_msgs::JointStateConstPtr& jointStates);

	/**
	 * @brief Internal method for adding a DELETE marker for the given robot.
	 * @param robotIt Iterator pointing to a Robot.
	 * @param markerArray Marker array where the marker should be appended.
	 */
	void addDeletionMsg(const Robots::iterator& robotIt, visualization_msgs::MarkerArray& markerArray) const;

	/**
	 * @brief Publishes visualization markers for a robot.
	 * @param robtoIt Iterator pointing to the robot.
	 */
	void publishVisualization(const Robots::iterator& robotIt) const;
};

} /* namespace rviz_robot_marker */

#endif /* INCLUDE_RVIZROBOTMARKERPUBLISHER_H_ */
