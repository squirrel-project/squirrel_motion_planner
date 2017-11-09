#include <ros/ros.h>
#include <rviz_robot_marker/RvizRobotMarkerPublisher.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

namespace rviz_robot_marker {

RvizRobotMarkerPublisher::RvizRobotMarkerPublisher(const std::string& markerTopic) : nh(), highestRobotId(0) {
	markerPub = nh.advertise<visualization_msgs::MarkerArray>(markerTopic, 100, false);
}

RvizRobotMarkerPublisher::~RvizRobotMarkerPublisher() {
}

void RvizRobotMarkerPublisher::setRobotPose(const size_t& mid, const tf::StampedTransform& baseTransform, const sensor_msgs::JointStateConstPtr& jointStates) {
	Robots::iterator robotIt = robots.find(mid);
	if (robotIt == robots.end()) {
		throw std::invalid_argument("Called setRobotPose() with unknown robot id. Call addRobotFromDescription() first.");
	}
	if (jointStates->name.size() != jointStates->position.size()) {
		throw std::invalid_argument("In setRobotPose(): size of name and position vectors are different");
	}
	robotIt->second->jointStates = jointStates;
	robotIt->second->baseTransform = baseTransform;

	if (!robotIt->second->hide) {
		publishVisualization(robotIt);
	}
}

void RvizRobotMarkerPublisher::publishVisualization(const Robots::iterator& robotIt) const {
	// Compute link poses from joint states and fill a local TF buffer
	tf::Transformer tf;
	computeChildTransforms(robotIt->second->jointStates, robotIt->second->kdlTree.getRootSegment(), tf);
	const ros::Time time = robotIt->second->jointStates->header.stamp;

	// Create marker messages
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;
	marker.header.stamp = time;
	if (robotIt->second->baseTransform.frame_id_.empty()) {
		marker.header.frame_id = robotIt->second->urdf->getRoot()->name;
	} else {
		marker.header.frame_id = robotIt->second->baseTransform.frame_id_;
	}
	marker.action = visualization_msgs::Marker::ADD;
	marker.ns = robotIt->second->markerNS;
	marker.id = 0;

	std::vector<boost::shared_ptr<urdf::Link> > links;
	robotIt->second->urdf->getLinks(links);
	for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator linkIt = links.begin(); linkIt != links.end(); ++linkIt) {
		marker.color = robotIt->second->markerColor;
		if ((*linkIt)->visual && (*linkIt)->visual->geometry) {
			bool addMarker;
			// Convert URDF geometric primitives to visualization markers
			switch ((*linkIt)->visual->geometry->type) {
			case urdf::Geometry::MESH:
				marker.type = visualization_msgs::Marker::MESH_RESOURCE;
				{
					const boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>((*linkIt)->visual->geometry);
					marker.mesh_resource = mesh->filename;
					marker.scale.x = mesh->scale.x;
					marker.scale.y = mesh->scale.y;
					marker.scale.z = mesh->scale.z;
				}
				marker.mesh_use_embedded_materials = true;
				addMarker = true;
				break;
			case urdf::Geometry::BOX:
				marker.type = visualization_msgs::Marker::CUBE;
				{
					const boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>((*linkIt)->visual->geometry);
					marker.scale.x = box->dim.x;
					marker.scale.y = box->dim.y;
					marker.scale.z = box->dim.z;
				}
				addMarker = true;
				break;
			case urdf::Geometry::CYLINDER:
				marker.type = visualization_msgs::Marker::CYLINDER;
				{
					const boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>((*linkIt)->visual->geometry);
					marker.scale.x = 2. * cylinder->radius;
					marker.scale.y = 2. * cylinder->radius;
					marker.scale.z = cylinder->length;
				}
				addMarker = true;
				break;
			case urdf::Geometry::SPHERE:
				marker.type = visualization_msgs::Marker::SPHERE;
				{
					const boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>((*linkIt)->visual->geometry);
					marker.scale.x = 2. * sphere->radius;
					marker.scale.y = 2. * sphere->radius;
					marker.scale.z = 2. * sphere->radius;
				}
				addMarker = true;
				break;

			default:
				ROS_WARN("Unknown marker type for link %s", (*linkIt)->name.c_str());
				addMarker = false;
			}
			if (addMarker) {
				if ((*linkIt)->visual->material) {
					switch (robotIt->second->colorMode) {
					case REPLACE_MESH_COLOR:
						marker.color.r = (*linkIt)->visual->material->color.r;
						marker.color.g = (*linkIt)->visual->material->color.g;
						marker.color.b = (*linkIt)->visual->material->color.b;
						marker.color.a = (*linkIt)->visual->material->color.a;
						break;
					case FORCE_COLOR:
						marker.color = robotIt->second->markerColor;
						break;
					case BLEND:
						{
							const double a1 = robotIt->second->markerColor.a;
							const double a2 = (*linkIt)->visual->material->color.a;
							const double a12Rec = 1. / (a1 + a2);
							marker.color.a = 0.5 * (a1 + a2);
							marker.color.r = a12Rec * (a1 * robotIt->second->markerColor.r + a2 * (*linkIt)->visual->material->color.r);
							marker.color.g = a12Rec * (a1 * robotIt->second->markerColor.g + a2 * (*linkIt)->visual->material->color.g);
							marker.color.b = a12Rec * (a1 * robotIt->second->markerColor.b + a2 * (*linkIt)->visual->material->color.b);
						}
						break;
					}
				}

				try {
				if (robotIt->second->baseTransform.frame_id_.empty()) {
					// No base transform: lookup intra-robot transform and publish in base link frame
					tf::StampedTransform baseToLink;
					tf.lookupTransform(marker.header.frame_id, (*linkIt)->name, time, baseToLink);
					tf::poseTFToMsg(baseToLink, marker.pose);
				} else {
					// Valid base transform: lookup intra-robot transform and chain it with the baseTransform
					tf::StampedTransform baseToLink;
					tf.lookupTransform(robotIt->second->baseTransform.child_frame_id_, (*linkIt)->name, time, baseToLink);
					tf::poseTFToMsg(robotIt->second->baseTransform * baseToLink, marker.pose);
				}
				markers.markers.push_back(marker);
				robotIt->second->highestMarkerId = marker.id;
				++marker.id;
				} catch(const tf2::LookupException& e) {
					ROS_ERROR("%s\nExisting frames are: %s", e.what(), tf.allFramesAsString().c_str());
				}
			}
		}
	}
	markerPub.publish(markers);
}

void RvizRobotMarkerPublisher::clear() {
	visualization_msgs::MarkerArray markers;
	for (Robots::iterator robotIt = robots.begin(); robotIt != robots.end(); ++robotIt) {
		if (robotIt->second->jointSubscriber) {
			robotIt->second->jointSubscriber->shutdown();
		}
		addDeletionMsg(robotIt, markers);
	}
	markerPub.publish(markers);
	robots.clear();
}

void RvizRobotMarkerPublisher::deleteRobot(const size_t& id) {
	Robots::iterator robotIt = robots.find(id);
	if (robotIt == robots.end()) {
		throw std::invalid_argument("Called deleteRobot() with unknown robot id.");
	}

	if (robotIt->second->jointSubscriber) {
		robotIt->second->jointSubscriber->shutdown();
	}

	visualization_msgs::MarkerArray markers;
	addDeletionMsg(robotIt, markers);
	markerPub.publish(markers);
	robots.erase(robotIt);
}

void RvizRobotMarkerPublisher::addDeletionMsg(const Robots::iterator& robotIt, visualization_msgs::MarkerArray& markerArray) const {
	visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::DELETE;
	marker.ns = robotIt->second->markerNS;
	for (size_t j = 0; j <= robotIt->second->highestMarkerId; ++j) {
		marker.id = j;
		markerArray.markers.push_back(marker);
	}
}


void RvizRobotMarkerPublisher::jointStateCallback(const size_t& id, const sensor_msgs::JointStateConstPtr& jointStates) {
	try {
		setRobotPose(id, tf::StampedTransform(), jointStates);
	} catch(const std::invalid_argument& e) {
		ROS_ERROR("Joint state callback: %s", e.what());
	}
}

const size_t RvizRobotMarkerPublisher::addRobotFromDescription(const std::string& param) {
	boost::shared_ptr<Robot> robot(new Robot());
	UrdfModelMap::iterator modelIt = urdfModels.find(param);
	if (modelIt == urdfModels.end()) {
		std::string robotDescription;
		if (nh.getParam(param, robotDescription)) {
			boost::shared_ptr<const urdf::ModelInterface> model = urdf::parseURDF(robotDescription);
			modelIt = urdfModels.insert(std::make_pair(param, model)).first;
		} else {
			throw std::invalid_argument("Could not find robot_description parameter");
		}
	}
	robot->urdf = modelIt->second;
	if (!kdlModelFromURDF(robot->urdf, robot)) {
		throw std::invalid_argument("Could not generate KDL tree from robot description");
	}

	robot->highestMarkerId = 0;
	robot->markerColor.r = 0.5;
	robot->markerColor.g = 0.5;
	robot->markerColor.b = 0.5;
	robot->markerColor.a = 1.0;
	robot->colorMode = REPLACE_MESH_COLOR;
	robot->hide = false;

	const size_t id = ++highestRobotId;
	char buffer[128];
	snprintf(buffer, sizeof(buffer), "robot_%zu", id);
	robot->markerNS = buffer;
	robots.insert(std::make_pair(id, robot));
	return id;
}


const size_t RvizRobotMarkerPublisher::addRobotWithJointListener(const std::string& topic, const std::string& param) {
	const size_t robotId = addRobotFromDescription(param);
	robots[robotId]->jointSubscriber.reset(new ros::Subscriber(nh.subscribe<sensor_msgs::JointState>(topic, 10, boost::bind(&RvizRobotMarkerPublisher::jointStateCallback, this, robotId, _1))));
	return robotId;
}

bool RvizRobotMarkerPublisher::kdlModelFromURDF(const boost::shared_ptr<const urdf::ModelInterface>& model, boost::shared_ptr<Robot> robot) {
	if (!kdl_parser::treeFromUrdfModel(*model, robot->kdlTree)){
		throw std::invalid_argument("Failed to extract kdl tree from xml robot description");
		return false;
	}
	return true;
}


bool RvizRobotMarkerPublisher::computeChildTransforms(const sensor_msgs::JointStateConstPtr& jointStates, const KDL::SegmentMap::const_iterator& segment, tf::Transformer& tf) const {
	for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator child = segment->second.children.begin();
		      child != segment->second.children.end(); ++child) {
		double jnt_p = 0.0;
		if ((*child)->second.segment.getJoint().getType() != KDL::Joint::None) {
			for (size_t i = 0; i < jointStates->name.size(); ++i) {
				if (jointStates->name[i] == (*child)->second.segment.getJoint().getName()) {
					jnt_p = jointStates->position[i];
					break;
				}
			}
		}
		const KDL::Frame frame = (*child)->second.segment.pose(jnt_p);
		tf::StampedTransform transform;
		tf::transformKDLToTF(frame, transform);
		transform.stamp_ = jointStates->header.stamp;
		transform.frame_id_ = segment->second.segment.getName();
		transform.child_frame_id_ = (*child)->second.segment.getName();
		tf.setTransform(transform, ros::this_node::getName());

		computeChildTransforms(jointStates, *child, tf);
	}
	return true;
}

void RvizRobotMarkerPublisher::setRobotColor(const size_t& id, const std_msgs::ColorRGBA& color, const ColorMode& colorMode) {
	Robots::iterator robotIt = robots.find(id);
	if (robotIt == robots.end()) {
		throw std::invalid_argument("Called setRobotColor() with unknown robot id. Call addRobotFromDescription() first.");
	}

	robots[id]->markerColor = color;
	robots[id]->colorMode = colorMode;
}

void RvizRobotMarkerPublisher::setRobotName(const size_t& id, const std::string& name) {
	Robots::iterator robotIt = robots.find(id);
	if (robotIt == robots.end()) {
		throw std::invalid_argument("Called setRobotColor() with unknown robot id. Call addRobotFromDescription() first.");
	}
	robots[id]->markerNS = name;
}

void RvizRobotMarkerPublisher::showRobot(const size_t& id) {
	Robots::iterator robotIt = robots.find(id);
	if (robotIt == robots.end()) {
		throw std::invalid_argument("Called setRobotColor() with unknown robot id. Call addRobotFromDescription() first.");
	}
	robotIt->second->hide = false;
	publishVisualization(robotIt);
}


void RvizRobotMarkerPublisher::hideRobot(const size_t& id) {
	Robots::iterator robotIt = robots.find(id);
	if (robotIt == robots.end()) {
		throw std::invalid_argument("Called setRobotColor() with unknown robot id. Call addRobotFromDescription() first.");
	}
	robotIt->second->hide = true;
	visualization_msgs::MarkerArray markers;
	addDeletionMsg(robotIt, markers);
	markerPub.publish(markers);
}

} /* namespace rviz_robot_marker */
