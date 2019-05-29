#ifndef BASICTASK_H
#define BASICTASK_H

/*
    BASIC TASK CLASS
    This object is a container for jacobian and other entities of a task.
*/

// Basic Includes
#include <Eigen/Dense>

namespace adaptive_grasping {

class basicTask {

public:

    // Default Constructor
    basicTask();

    // Overloaded Constructor
    basicTask(Eigen::MatrixXd task_jacobian, int task_priority);

    // Destructor
    ~basicTask();

    // Public Auxiliary Fuctions
    void set_task_jacobian(Eigen::MatrixXd jacobian);
    void set_task_priority(int priority);
    Eigen::MatrixXd get_task_jacobian();

private:

    // Task Jacobian
    Eigen::MatrixXd task_jacobian_;

    // Task Priority
    int task_priority_;

};

}


#endif // BASICTASK_H
