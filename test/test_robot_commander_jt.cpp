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
#include "robotCommanderJT.h"


int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "robot_commander_jt_server");
	ros::NodeHandle rc_jt_serv_nh;

    std::string hand_topic = "/right_hand/joint_trajectory_controller/follow_joint_trajectory/";
    std::string arm_topic = "/right_arm/joint_trajectory_controller/follow_joint_trajectory/";
    std::vector<std::string> joint_names_vec;
    joint_names_vec.push_back("right_arm_a1_joint");
    joint_names_vec.push_back("right_arm_a2_joint");
    joint_names_vec.push_back("right_arm_e1_joint");
    joint_names_vec.push_back("right_arm_a3_joint");
    joint_names_vec.push_back("right_arm_a4_joint");
    joint_names_vec.push_back("right_arm_a5_joint");
    joint_names_vec.push_back("right_arm_a6_joint");
    joint_names_vec.push_back("right_hand_synergy_joint");

    ros::Duration header_dur(0, 1000000);
    ros::Duration exec_wait_dur(10, 0);
    ros::Duration dt(1, 0);

    // Creating the commander class
    adaptive_grasping::robotCommanderJT robot_commander(hand_topic, arm_topic, joint_names_vec,
        header_dur, exec_wait_dur, dt);
	
	// Success message
	ROS_INFO("The Robot Commander JT Server ready to process requests!");
	ROS_DEBUG_STREAM("DEBUG ACTIVATED!");

	// Spin
	ros::spin();

}