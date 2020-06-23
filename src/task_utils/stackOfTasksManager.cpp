//
// Created by George on 20/11/19.
//

#include "task_utils/stackOfTasksManager.h"

// ROS Includes
#include <ros/ros.h>

#define DEBUG           0           // Prints out additional info (additional to ROS_DEBUG)
#define USE_DAMPING     0           // If true the manager makes use of damping while pseudo inverting

/**
* @brief The following are functions of the class stackOfTasksManager.
*
*/

// Default Constructor
stackOfTasksManager::stackOfTasksManager(){
	// Nothing to do here
}

// Overloaded Constructor 1
stackOfTasksManager::stackOfTasksManager(int dim_config_space, double lambda_max, double epsilon) {
	// Setting the dimension of the configuration space of the tasks and the RP constants
	this->dim_config_space_ = dim_config_space;
	this->lambda_max_ = lambda_max;
	this->epsilon_ = epsilon;
}

// Overloaded Constructor 2
stackOfTasksManager::stackOfTasksManager(int dim_config_space, double lambda_max, double epsilon, std::vector<basicTask> starting_task_set) {
	// Setting the dimension of the configuration space of the tasks and the RP constants
	this->dim_config_space_ = dim_config_space;
	this->lambda_max_ = lambda_max;
	this->epsilon_ = epsilon;

	// Inserting the tasks
	if (!this->insert_tasks(starting_task_set)) ros::shutdown();
}

// Destructor
stackOfTasksManager::~stackOfTasksManager() {
	// Nothing to do here for now
}

// Auxiliary Public Functions
bool stackOfTasksManager::set_basics(int dim_config_space, double lambda_max, double epsilon) {
	// Setting the dimension of the configuration space of the tasks and the RP constants
	this->dim_config_space_ = dim_config_space;
	this->lambda_max_ = lambda_max;
	this->epsilon_ = epsilon;

	ROS_INFO_STREAM("This RP Manager has dim_config_space_ " << this->dim_config_space_ << " lambda_max_ " << this->lambda_max_ << " epsilon " << this->epsilon_ << ".");
}


bool stackOfTasksManager::insert_tasks(std::vector<basicTask> tasks) {
	// Checking that all the tasks of the task set have the same configuration space dimensions
	bool tasks_ok = true;
	for (std::vector<basicTask>::iterator it = tasks.begin(); it != tasks.end(); ++it) {
		if (it->get_task_jacobian().cols() != this->dim_config_space_) {
			ROS_ERROR_STREAM("The " << it - tasks.begin() <<
			                        "th task has a number of columns (" << it->get_task_jacobian().cols() <<
			                        ") != configuration space dimension (" << this->dim_config_space_ << ")! This won't work anymore!");
			tasks_ok = false;
		}
	}

	// If no consistency between tasks return false
	if (!tasks_ok) return false;

	// Appending the input task vector to the existing task set
	this->task_set_.insert(this->task_set_.end(), tasks.begin(), tasks.end());
}

void stackOfTasksManager::remove_task(int task_priority) {
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

void stackOfTasksManager::reorder_set() {
	std::sort(this->task_set_.begin(), this->task_set_.end());
}

void stackOfTasksManager::clear_set() {
	this->task_set_.clear();
}

void stackOfTasksManager::print_set() {
	ROS_INFO_STREAM("The task set is: ");
	for (std::vector<basicTask>::iterator it = this->task_set_.begin(); it != this->task_set_.end(); ++it) {
		std::cout << "---------------------" << std::endl;
		std::cout << "priority: " << it->get_task_priority() << std::endl;
		std::cout << "sec_priority: " << it->get_sec_priority() << std::endl;
		std::cout << "x_dot: \n" << it->get_task_x_dot() << std::endl;
		std::cout << "jacobian: \n" << it->get_task_jacobian() << std::endl;
	}
	std::cout << "---------------------" << std::endl;
}

void stackOfTasksManager::solve_inv_kin(Eigen::VectorXd &q_sol) {
	// Initializing the result vector
	Eigen::VectorXd q_res; q_res.resize(this->dim_config_space_); q_res.setZero();
	if (DEBUG) std::cout << "The dimension of the config space is " << this->dim_config_space_ << "." << std::endl;

	// Ordering the task set
	this->reorder_set();
	if (DEBUG) this->print_set();

	// Initialize projection matrices and the current jacobian and task vector
	int n_cols_init = this->task_set_.at(0).get_task_jacobian().cols();
	Eigen::MatrixXd P_i_1 = Eigen::MatrixXd::Identity(n_cols_init, n_cols_init);
	Eigen::MatrixXd J_i;
	Eigen::VectorXd x_dot_i;
	Eigen::MatrixXd JP_i_pinv;

	if (DEBUG) std::cout << "Starting SOT with q_res = " << q_res << "." << std::endl;

	// Recursion loop (ref stack of tasks paper)
	for (auto it : this->task_set_) {
		// Getting the needed variables
		x_dot_i = it.get_task_x_dot();
		J_i = it.get_task_jacobian();

		// Computing current solution
		if (USE_DAMPING) {
			JP_i_pinv = damped_pseudo_inv(J_i * P_i_1, this->lambda_max_, this->epsilon_);
		} else {
			JP_i_pinv = trunk_pseudo_inv(J_i * P_i_1, this->epsilon_);
		}
		q_res = q_res + JP_i_pinv * (x_dot_i - J_i * q_res);

		// Debug couts
		if (DEBUG) {
			std::cout << "x_dot_i: \n" << x_dot_i << std::endl;
			std::cout << "J_i: \n" << J_i << std::endl;
			std::cout << "P_i_1: \n" << P_i_1 << std::endl;
			std::cout << "JP_i_pinv: \n" << JP_i_pinv << std::endl;
			std::cout << "q_res: \n" << q_res << std::endl;
		}

		// Updating the projection matrix
		P_i_1 = P_i_1 - JP_i_pinv * J_i * P_i_1;

	}

	if (DEBUG) std::cout << "Ending SOT with q_res = " << q_res << "." << std::endl;

	// Returning
	q_sol = q_res;
}
