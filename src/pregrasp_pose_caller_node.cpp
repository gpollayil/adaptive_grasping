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

// CUSTOM INCLUDES
#include "panda_softhand_control/PandaSoftHandClient.h"

// DEFINES
#define DEBUG       1       // Prints out additional info


int main(int argc, char** argv){

    // Joint configs
    std::vector<double> home_joints = {0.5967, 0.3222, -0.2777, -1.2602, 0.4703, 3.2581, -0.5125};
    std::vector<double> grasp_joints = {-0.2506, 0.5673, -0.4563, -1.8466, 0.2444, 2.3360, -0.9379};
    std::vector<double> other_joints = {0.3712, 0.6079, -0.1783, -2.0502, 0.2459, 2.7280, 0.0787};

	// Initializing ROS node
	ros::init(argc, argv, "adaptive_grasping_pregrasp_pose_caller_node");
	ros::NodeHandle service_caller_nh;

    // The Panda SoftHand Client
    PandaSoftHandClient panda_softhand_client;
    // Initializing Panda SoftHand Client
    panda_softhand_client.initialize(service_caller_nh);

    // Asking for what to do
    std::cout << " Enter 0 for home or 1 for grasp: ";
    int input;
    std::cin >> input;

    if ( input == 0 ) {

        // 1) Going to home configuration
        if(!panda_softhand_client.call_joint_service(home_joints)){
            ROS_ERROR("Could not go to the specified home joint configuration.");
            return 0;
        }

    } else if ( input == 1 ) {

        // 1) Going to grasp configuration
        if(!panda_softhand_client.call_joint_service(grasp_joints)){
            ROS_ERROR("Could not go to the specified grasp joint configuration.");
            return 0;
        }

        // 1) Going to other configuration
        if(!panda_softhand_client.call_joint_service(other_joints)){
            ROS_ERROR("Could not go to the specified other joint configuration.");
            return 0;
        }

    } else {

        ROS_ERROR("Don't know this input! Returning!");
        return 0;

    }

    // Success message
	ROS_INFO("\nTerminating Pregrasp caller!");

    return 0;
}