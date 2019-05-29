#include "reversePriorityManager.h"

// ROS Includes
#include <ros/ros.h>

#define DEBUG   0           // Prints out additional info (additional to ROS_DEBUG)

/**
* @brief The following are functions of the class reversePriorityManager.
*
*/

using namespace adaptive_grasping;

// Default Constructor
reversePriorityManager::reversePriorityManager(int dim_config_space) {

    // Setting the dimension of the configuration space of the tasks
    this->dim_config_space_ = dim_config_space;

}

// Overloaded Constructor
reversePriorityManager::reversePriorityManager(int dim_config_space, std::vector<basicTask> starting_task_set) {

    // Setting the dimension of the configuration space of the tasks
    this->dim_config_space_ = dim_config_space;

    // Checking that all the tasks of the task set have the same configuration space dimensions
    bool tasks_ok = true;
    for (std::vector<basicTask>::iterator it = starting_task_set.begin(); it != starting_task_set.end(); ++it) {
        if (it->get_task_jacobian().cols() != this->dim_config_space_) {
            ROS_ERROR_STREAM("The " << it - starting_task_set.begin() <<
                             "th task has a number of columns != configuration space dimension! This won't work anymore!");
            tasks_ok = false;
        }
    }

    if (!tasks_ok) ros::shutdown();
    else this->insert_tasks(starting_task_set);

}

// Auxiliary Public Functions
void reversePriorityManager::insert_tasks(std::vector<basicTask> tasks){

}
