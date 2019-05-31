/* For testing reversePriorityManager class */

// Basic Includes
#include <iostream>
#include <ros/ros.h>


#include "basicTask.h"
#include "reversePriorityManager.h"

using namespace adaptive_grasping;

int main(int argc, char **argv) {

    // Starting the test node
    std::cout<<std::endl;
    std::cout<<"|Adaptive Grasping| -> Testing Reverse Priority!"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_reverse_priority");
    ros::NodeHandle nh;

    // Creating the objects
    std::vector<basicTask> task_vec;
    reversePriorityManager rp_manager;



}
