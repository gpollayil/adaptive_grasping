/*  Main file of Adaptive Grasping service
    It creates a ros node which has the adaptive object

    authors: George Jose Pollayil, Mathew Jose Pollayil
*/

//BASIC INCLUDES
#include <sstream>

// ROS INCLUDES
#include <ros/ros.h>

// CLASS INCLUDES
#include "adaptiveGrasper.h"

// DEFINES
#define DEBUG       1       // Prints out additional info


int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "adaptive_grasping_node");
	ros::NodeHandle adaptive_nh;

    // Creating the adaptive grasper class
    adaptive_grasping::adaptiveGrasper adaptive_grasper;

    // Initializing with a vector containing the names of items to be parsed
    std::vector<std::string> param_names;
    param_names.push_back("touch_topic_name");
    param_names.push_back("link_names_map");
    param_names.push_back("params_map");
    param_names.push_back("joints_num");
    param_names.push_back("h_matrix");
    param_names.push_back("a_tilde_matrix");
    param_names.push_back("x_d");
    param_names.push_back("spin_rate");
    param_names.push_back("object_topic_name");
    param_names.push_back("scaling");
    param_names.push_back("p_vector");
    param_names.push_back("syn_thresh");

    adaptive_grasper.initialize(param_names);

    // Printing the parsed parameters
    if(DEBUG) adaptive_grasper.printParsed();

    // Starting message
	ROS_INFO("\nThe Adaptive Grasper is starting to spin!");
	ROS_DEBUG_STREAM("DEBUG ACTIVATED!");

    // Starting to spin
    adaptive_grasper.spinGrasper();

    // Success message
	ROS_INFO("\nTerminating Adaptive Grasper!");

	// Spin
	ros::spin();

}