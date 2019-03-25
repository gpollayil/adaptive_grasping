/*  Main file of Adaptive Grasping service
    It creates a ros node which has the adaptive object

    authors: George Jose Pollayil, Mathew Jose Pollayil
*/

//BASIC INCLUDES
#include <sstream>

// ROS INCLUDES
#include <ros/ros.h>

// CLASS INCLUDES
#include "fullGrasper.h"

// DEFINES
#define DEBUG       1       // Prints out additional info


int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "adaptive_grasping_fg_node");
	ros::NodeHandle adaptive_nh;

    // Params for building full_grasper
    std::string arm_ns = "panda_arm";
    std::string hand_ns = "right_hand";
    std::vector<std::string> normal_controllers_names;
    normal_controllers_names.push_back("joint_trajectory_controller");
    normal_controllers_names.push_back("joint_trajectory_controller");
    std::vector<std::string> velocity_controllers_names;
    velocity_controllers_names.push_back("cartesian_velocity_controller");
    velocity_controllers_names.push_back("velocity_controller");

    // Creating the adaptive grasper class
    adaptive_grasping::fullGrasper full_grasper(arm_ns, hand_ns, normal_controllers_names, velocity_controllers_names);

    // Starting message
	ROS_INFO("\nThe Full Grasper is starting to spin!");
	ROS_DEBUG_STREAM("DEBUG ACTIVATED!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(5);
    spinner.start();

    // Waiting until termination
    ros::waitForShutdown();

    // Success message
	ROS_INFO("\nTerminating Full Grasper!");

    // Teminating
    spinner.stop();

    return 0;
}