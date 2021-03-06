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
	ros::init(argc, argv, "adaptive_grasping_ag_node");
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
    adaptive_grasping::adaptiveGrasper adaptive_grasper;

    // Initializing with a vector containing the names of items to be parsed
    // For adaptive_grasper
    std::vector<std::string> param_names;
    param_names.push_back("touch_topic_name");
    param_names.push_back("link_names_map");
    param_names.push_back("params_map");
    param_names.push_back("joints_num");
    param_names.push_back("h_matrix");
    param_names.push_back("k_matrix");
    param_names.push_back("x_d");
	param_names.push_back("f_d_d");
    param_names.push_back("spin_rate");
    param_names.push_back("object_topic_name");
	param_names.push_back("object_twist_topic_name");
    param_names.push_back("scaling");
    param_names.push_back("p_vector");
	param_names.push_back("touch_indexes");
    param_names.push_back("syn_thresh");
    param_names.push_back("relax_to_zero");
    param_names.push_back("touch_change");
    param_names.push_back("num_tasks");
    param_names.push_back("dim_tasks");
    param_names.push_back("prio_tasks");
    param_names.push_back("lambda_max");
    param_names.push_back("epsilon");

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

    return 0;
}
