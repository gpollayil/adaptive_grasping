#ifndef BASICTASK_H
#define BASICTASK_H

/*
    BASIC TASK CLASS
    This object is a container for jacobian and other entities of a task.
*/

// Basic Includes
#include <Eigen/Dense>

class basicTask {

public:

    // Default Constructor
    basicTask();

    // Overloaded Constructor
    basicTask(Eigen::VectorXd task_x_dot, Eigen::MatrixXd task_jacobian, int task_priority);

    // Destructor
    ~basicTask();

    // Public Auxiliary Fuctions
    void set_task_x_dot(Eigen::VectorXd x_dot);
    void set_task_jacobian(Eigen::MatrixXd jacobian);
    void set_task_priority(int priority);
    void set_sec_priority(int sec_priority);
    Eigen::VectorXd get_task_x_dot();
    Eigen::MatrixXd get_task_jacobian();
    inline int get_task_priority() {
        return this->task_priority_;
    }
    inline int get_sec_priority() {
        return this->sec_priority_;
    }

    /*
     Sort Predicate as operator< (This is needed to sort the task set in RP Manager)
     Indicates the if the current basicTask is greater or smaller than another basicTask
    */
    bool operator < (const basicTask& task) const {
        if (this->task_priority_ != task.task_priority_) {
            return (this->task_priority_ < task.task_priority_);
        } else {
            return (this->sec_priority_ < task.sec_priority_);
        }
    }

private:

    // Task velocity vector
    Eigen::VectorXd task_x_dot_;

    // Task Jacobian
    Eigen::MatrixXd task_jacobian_;

    // Task Priority
    int task_priority_;

    // Secondary Priority
    int sec_priority_;

};


#endif // BASICTASK_H
