#include "task_utils/basicTask.h"

// ROS Includes
#include <ros/ros.h>

#define DEBUG   0           // Prints out additional info (additional to ROS_DEBUG)

/**
* @brief The following are functions of the class basicTask.
*
*/

using namespace adaptive_grasping;

// Default Constructor
basicTask::basicTask(){

}

// Overloaded Constructor
basicTask::basicTask(Eigen::VectorXd task_x_dot, Eigen::MatrixXd task_jacobian, int task_priority) {
    // Dimensions consistency check
    if (task_x_dot.rows() != task_jacobian.rows()) ROS_ERROR("The dimensions of x_dot and jacobian of the task are inconsistent!");

    // Setting the stuff
    set_task_x_dot(task_x_dot);
    set_task_jacobian(task_jacobian);
    set_task_priority(task_priority);
    
    // Default secondary priority is zero (if other values needed, use public set function)
    this->sec_priority_ = 0;
}

// Destructor
basicTask::~basicTask() {
    // Nothing to do here for now
}

// Public Auxiliary Fuctions
void basicTask::set_task_x_dot(Eigen::VectorXd x_dot) {
    this->task_x_dot_ = x_dot;
}

void basicTask::set_task_jacobian(Eigen::MatrixXd jacobian) {
    this->task_jacobian_ = jacobian;
}

void basicTask::set_task_priority(int priority) {
    this->task_priority_ = priority;
}

void basicTask::set_sec_priority(int sec_priority) {
    this->sec_priority_ = sec_priority;
}

Eigen::VectorXd basicTask::get_task_x_dot() {
    return this->task_x_dot_;
}

Eigen::MatrixXd basicTask::get_task_jacobian() {
    return this->task_jacobian_;
}
