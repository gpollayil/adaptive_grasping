#ifndef REVERSEPRIORITYMANAGER_H
#define REVERSEPRIORITYMANAGER_H

/*
    REVERSE PRIORITY MANAGER CLASS
    This object is a manager for a set of tasks and performs reverse priority algorithm.
*/

// Basic Includes
#include <vector>

// Custom Includes
#include "basicTask.h"


namespace adaptive_grasping {

class reversePriorityManager {

public:

    // Default Constructor
    reversePriorityManager(int dim_config_space);

    // Overloaded Constructor
    reversePriorityManager(int dim_config_space, std::vector<basicTask> starting_task_set);

    // Destructor
    ~reversePriorityManager();

    // Auxiliary Public Functions
    void insert_tasks(std::vector<basicTask> tasks);    // Inserts a given task into the task set
    void remove_task(int task_priority);                // Removes a task with a certain priority from the task set
    void reorder_set();                                 // Reorders the task set from higher to lower priority (1, 2, 3, ...)
    void clear_set();                                   // Clears the task set
    void print_set();                                   // Prints to screen the whole task set
    Eigen::VectorXd solve_inv_kin();                    // Gives the reverse priority inverse kinematics solution for the task set

private:

    // Dimension of the configuration space
    int dim_config_space_;

    // Set of tasks ordered by priority
    std::vector<basicTask> task_set_;

    // Set of projection matrices Tk (ref. reverse priority)
    std::vector<Eigen::MatrixXd> t_proj_mat_set_;

    // Private Auxiliary Fuctions
    Eigen::MatrixXd damped_pseudo_inv(Eigen::MatrixXd input_mat, double damping_coeff, double epsilon);
    Eigen::MatrixXd rank_update(Eigen::MatrixXd J, Eigen::MatrixXd Jra_pinv);

};

}

#endif // REVERSEPRIORITYMANAGER_H
