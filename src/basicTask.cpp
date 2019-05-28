#include "basicTask.h"

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
basicTask::basicTask(Eigen::MatrixXd task_jacobian, int task_priority){
    set_task_jacobian(task_jacobian);
    set_task_priority(task_priority);
}

// Destructor
basicTask::~basicTask(){
    // Nothing to do here for now
}

// Public Auxiliary Fuctions
void basicTask::set_task_jacobian(Eigen::MatrixXd jacobian){
    this->task_jacobian_ = jacobian;
}

void basicTask::set_task_priority(int priority){
    this->task_priority_ = priority;
}
