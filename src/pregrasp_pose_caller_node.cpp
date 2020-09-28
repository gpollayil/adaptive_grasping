/*  Main file of Adaptive Grasping service
    It creates a ros node which has the adaptive object

    authors: George Jose Pollayil, Mathew Jose Pollayil
*/

//BASIC INCLUDES
#include <sstream>

// ROS INCLUDES
#include <ros/ros.h>

// MSG INCLUEDS
#include "std_srvs/SetBool.h"

// DEFINES
#define DEBUG       1       // Prints out additional info


int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "adaptive_grasping_pregrasp_pose_caller_node");
	ros::NodeHandle service_caller_nh;

    // Params for building full_grasper
    std::string pre_grasp_service_name = "/pregrasp_task_service";
    
    // Creating the service clients
    ros::ServiceClient pre_grasp_client = service_caller_nh.serviceClient<std_srvs::SetBool>(pre_grasp_service_name);

    // Creating a default true message
    std_srvs::SetBool def_bool_srv;
    def_bool_srv.request.data = true;

    // Starting message
	ROS_INFO("\nThe Service caller is starting to spin!");
	ROS_DEBUG_STREAM("DEBUG ACTIVATED!");

    // Calling the services in sequence
    
    // Pregrasp
    if (pre_grasp_client.call(def_bool_srv)) {
        ROS_INFO("Pregrasp success!");
    } else {
        ROS_ERROR("Failed to call pregrasp!");
    }

    // Success message
	ROS_INFO("\nTerminating Service caller!");

    return 0;
}