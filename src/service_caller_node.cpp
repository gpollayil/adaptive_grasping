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
	ros::init(argc, argv, "adaptive_grasping_service_caller_node");
	ros::NodeHandle service_caller_nh;

    // Params for building full_grasper
    std::string pre_grasp_service_name = "/pregrasp_task_service";
    std::string switch_pos2vel_service_name = "/switch_pos2vel_service";
    std::string adaptive_service_name = "/adaptive_task_service";
    std::string switch_vel2pos_service_name = "/switch_vel2pos_service";
    std::string postgrasp_service_name = "/postgrasp_task_service";
    
    // Creating the service clients
    ros::ServiceClient pre_grasp_client = service_caller_nh.serviceClient<std_srvs::SetBool>(pre_grasp_service_name);
    ros::ServiceClient switch_pos2vel_client = service_caller_nh.serviceClient<std_srvs::SetBool>(switch_pos2vel_service_name);
    ros::ServiceClient adaptive_client = service_caller_nh.serviceClient<std_srvs::SetBool>(adaptive_service_name);
    ros::ServiceClient switch_vel2pos_client = service_caller_nh.serviceClient<std_srvs::SetBool>(switch_vel2pos_service_name);
    ros::ServiceClient postgrasp_client = service_caller_nh.serviceClient<std_srvs::SetBool>(postgrasp_service_name);

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

    // Switch pos to vel
    if (switch_pos2vel_client.call(def_bool_srv)) {
        ROS_INFO("Switching to velocity from position success!");
    } else {
        ROS_ERROR("Failed to call switch to velocity!");
    }

    // Adaptive grasp
    if (adaptive_client.call(def_bool_srv)) {
        ROS_INFO("Adaptive grasp success!");
    } else {
        ROS_ERROR("Failed to call adaptive grasp !");
    }

    // Switch vel to pos
    if (switch_vel2pos_client.call(def_bool_srv)) {
        ROS_INFO("Switching to position from velocity success!");
    } else {
        ROS_ERROR("Failed to call switch to position!");
    }

    // Postgrasp
    if (postgrasp_client.call(def_bool_srv)) {
        ROS_INFO("Postgrasp_client success!");
    } else {
        ROS_ERROR("Failed to call postgrasp!");
    }

    // Success message
	ROS_INFO("\nTerminating Service caller!");

    return 0;
}