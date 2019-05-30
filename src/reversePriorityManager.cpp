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
void reversePriorityManager::insert_tasks(std::vector<basicTask> tasks) {
    // Appending the input task vector to the existing task set
    this->task_set_.insert(this->task_set_.end(), tasks.begin(), tasks.end());
}

void reversePriorityManager::remove_task(int task_priority) {
    // Finding the index of the element with given task_priority
    int index = 0;
    for (std::vector<basicTask>::iterator it = this->task_set_.begin(); it != this->task_set_.end(); ++it) {
        if (it->get_task_priority() == task_priority) {
            break;
        } else {
            index++;
        }
    }

    // Removing the element with found index
    this->task_set_.erase(this->task_set_.begin() + index);
}

void reversePriorityManager::reorder_set() {
    std::sort(this->task_set_.begin(), this->task_set_.end());
}

void reversePriorityManager::clear_set() {
    this->task_set_.clear();
}

void reversePriorityManager::print_set() {
    ROS_INFO_STREAM("The task set is: ");
    for (std::vector<basicTask>::iterator it = this->task_set_.begin(); it != this->task_set_.end(); ++it) {
        std::cout << "---------------------" << std::endl;
        std::cout << "priority: " << it->get_task_priority() << std::endl;
        std::cout << "jacobian: " << it->get_task_jacobian() << std::endl;
    }
    std::cout << "---------------------" << std::endl;
}

Eigen::VectorXd reversePriorityManager::solve_inv_kin() {

}

// Private Auxiliary Fuctions
Eigen::MatrixXd reversePriorityManager::damped_pseudo_inv(Eigen::MatrixXd input_mat, double damping_coeff, double epsilon) {

}

Eigen::MatrixXd reversePriorityManager::rank_update(Eigen::MatrixXd J, Eigen::MatrixXd Jra_pinv) {

}
