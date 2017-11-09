#include <ros/ros.h>
#include <rviz_robot_marker/RvizRobotMarkerPublisher.h>

using namespace rviz_robot_marker;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "publish_marker");
    std::string param = "/robot_description";
    std::list<std::string> topics;
    bool printHelp = false;
    for (size_t i = 1; i < argc; ++i) {
    	if (strcmp(argv[i], "--help") == 0) {
    		printHelp = true;
    		break;
    	} else if (strcmp(argv[i], "--param") == 0) {
    		if (i == argc - 1) {
    		    fputs("Param needs to be followed by the parameter name providing the robot description\n", stderr);
    		    printHelp = true;
    		    break;
    		}
    		param = argv[i + 1];
    		++i;
    	} else if (strcmp(argv[i], "--topic") == 0) {
    		if (i == argc - 1) {
    		    fputs("Param needs to be followed by the parameter name providing the robot description\n", stderr);
    		    printHelp = true;
    		    break;
    		}
    		topics.push_back(argv[i + 1]);
    		++i;
    	}
    }
    if (printHelp) {
    	fprintf(stderr, "Usage: %s [--param robot_description] [--topic joint_state_topic] ...\n\n", argv[0]);
    	fprintf(stderr, "For each sensor_msgs::JointState topic given by --topic, a robot marker will be created\n");
    	fprintf(stderr, "and updated when new messages arrive.\n");
    	return 1;
    }

    try {
    	RvizRobotMarkerPublisher publisher;
    	for (std::list<std::string>::const_iterator topicIt = topics.begin(); topicIt != topics.end(); ++topicIt) {
    		publisher.addRobotWithJointListener(*topicIt, param);
    	}
    	ros::spin();
    } catch (const std::invalid_argument& e) {
    	fprintf(stderr, "Could not run rviz robot marker publisher: %s\n", e.what());
    	ROS_FATAL("Could not run rviz robot marker publisher: %s\n", e.what());
    	return 1;
    }
    return 0;
}
