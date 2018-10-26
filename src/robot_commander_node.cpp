/**
* @brief    This ROS node provides a service to execute velocity commands on
*           on KUKA + SoftHand
*
*/

//BASIC INCLUDES
#include <sstream>

// ROS INCLUDES
#include <ros/ros.h>

// CLASS INCLUDES
#include "robotCommander.h"


int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "robot_commander_server");
	ros::NodeHandle rc_jt_serv_nh;

    std::string hand_topic = "/right_hand/velocity_controller/command/";
    std::string arm_topic = "/right_arm/twist_controller/command/";

    // Creating the commander class
    adaptive_grasping::robotCommander robot_commander(hand_topic, arm_topic);
	
	// Success message
	ROS_INFO("The Robot Commander Server ready to process requests!");
	ROS_DEBUG_STREAM("DEBUG ACTIVATED!");

	// Spin
	ros::spin();

}