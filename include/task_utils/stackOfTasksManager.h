//
// Created by George on 20/11/19.
//

#ifndef SRC_STACKOFTASKSMANAGER_H
#define SRC_STACKOFTASKSMANAGER_H

/*
    STACK OF TASKS MANAGER CLASS
    This object is a manager for a set of tasks and performs stack of tasks algorithm.
*/

// Basic Includes
#include <vector>

// Custom Includes
#include "basicTask.h"
#include "utils/inversion_utilities.h"

class stackOfTasksManager {

public:

	// Default Constructor
	stackOfTasksManager();

	// Overloaded Constructor 1
	stackOfTasksManager(int dim_config_space, double lambda_max, double epsilon);

	// Overloaded Constructor 2
	stackOfTasksManager(int dim_config_space, double lambda_max, double epsilon, std::vector<basicTask> starting_task_set);

	// Destructor
	~stackOfTasksManager();

	// Auxiliary Public Functions
	bool set_basics(int dim_config_space, double lambda_max, double epsilon);           // Sets basic stuff as in overloaded constructor 1
	bool insert_tasks(std::vector<basicTask> tasks);                                    // Inserts a given task into the task set
	void remove_task(int task_priority);                                                // Removes a task with a certain priority from the task set
	void reorder_set();                                                                 // Reorders the task set from higher to lower priority (1, 2, 3, ...)
	void clear_set();                                                                   // Clears the task set
	void print_set();                                                                   // Prints to screen the whole task set
	void solve_inv_kin(Eigen::VectorXd &q_sol);                                         // Gives the stack of tasks inverse kinematics solution for the task set

private:

	// Constants of the task space and for SOT algorithm
	int dim_config_space_;                                  // Dimension of the configuration space
	double lambda_max_;                                     // Damping for pseudo inversion
	double epsilon_;                                        // Bound for pseudo inversion

	// Set of tasks ordered by priority
	std::vector<basicTask> task_set_;

	// Set of projection matrices Pk+1 (ref. stack of tasks)
	std::vector<Eigen::MatrixXd> proj_mat_set_;

};

#endif //SRC_STACKOFTASKSMANAGER_H
