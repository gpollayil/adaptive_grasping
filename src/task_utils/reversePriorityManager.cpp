#include "task_utils/reversePriorityManager.h"

// ROS Includes
#include <ros/ros.h>

#define DEBUG           0           // Prints out additional info (additional to ROS_DEBUG)

/**
* @brief The following are functions of the class reversePriorityManager.
*
*/

using namespace adaptive_grasping;

// Default Constructor
reversePriorityManager::reversePriorityManager(){
    // Nothing to do here
}

// Overloaded Constructor 1
reversePriorityManager::reversePriorityManager(int dim_config_space, double lambda_max, double epsilon) {
    // Setting the dimension of the configuration space of the tasks and the RP constants
    this->dim_config_space_ = dim_config_space;
    this->lambda_max_ = lambda_max;
    this->epsilon_ = epsilon;
}

// Overloaded Constructor 2
reversePriorityManager::reversePriorityManager(int dim_config_space, double lambda_max, double epsilon, std::vector<basicTask> starting_task_set) {
    // Setting the dimension of the configuration space of the tasks and the RP constants
    this->dim_config_space_ = dim_config_space;
    this->lambda_max_ = lambda_max;
    this->epsilon_ = epsilon;

    // Inserting the tasks
    if (!this->insert_tasks(starting_task_set)) ros::shutdown();
}

// Destructor
reversePriorityManager::~reversePriorityManager() {
    // Nothing to do here for now
}

// Auxiliary Public Functions
bool reversePriorityManager::set_basics(int dim_config_space, double lambda_max, double epsilon) {
    // Setting the dimension of the configuration space of the tasks and the RP constants
    this->dim_config_space_ = dim_config_space;
    this->lambda_max_ = lambda_max;
    this->epsilon_ = epsilon;

    ROS_INFO_STREAM("This RP Manager has dim_config_space_ " << this->dim_config_space_ << " lambda_max_ " << this->lambda_max_ << " epsilon " << this->epsilon_ << ".");
}


bool reversePriorityManager::insert_tasks(std::vector<basicTask> tasks) {
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
        std::cout << "sec_priority: " << it->get_sec_priority() << std::endl;
        std::cout << "x_dot: \n" << it->get_task_x_dot() << std::endl;
        std::cout << "jacobian: \n" << it->get_task_jacobian() << std::endl;
    }
    std::cout << "---------------------" << std::endl;
}

bool reversePriorityManager::solve_inv_kin(Eigen::VectorXd &q_sol) {
    // Initializing the result vector
    Eigen::VectorXd q_res; q_res.resize(this->dim_config_space_); q_res.setZero();

    // Ordering the task set
    this->reorder_set();
    if (DEBUG || true) this->print_set();

    // Computing all the Projection matrices
    if (!this->compute_proj_mats()) {      // If this fails, return false and solution is zeros
        ROS_ERROR("Could not compute any task! Won't solve anything!");
        q_sol = q_res;
        return false;
    }

    // Size of task set
    auto task_set_dim = this->task_set_.size();

    // Creating vector of RP recursion and its last element is the initial guess
    std::vector<Eigen::VectorXd> q_vec;
    q_vec.resize(task_set_dim + 1);
    q_vec.at(int (task_set_dim)) = Eigen::VectorXd::Zero(this->dim_config_space_);

    // The RP recursion (ref. paper)
    for (int i = int (task_set_dim) - 1; i >= 0; i--) {
        // Computing the needed variables for recursion
        Eigen::VectorXd x_dot_i = this->task_set_.at(i).get_task_x_dot();
        Eigen::MatrixXd J_i = this->task_set_.at(i).get_task_jacobian();
        Eigen::MatrixXd P_i1 = this->proj_mat_set_.at(i+1);
        Eigen::MatrixXd pinv_J_i_P_i1 = damped_pseudo_inv((J_i*P_i1), this->lambda_max_, this->epsilon_);

        // Debug print outs
        if (DEBUG) {
            ROS_INFO_STREAM("The quantities for the " << i << "th recursion formula are: ");
            std::cout << "q_vec.at(" << i+1 << "): \n" << q_vec.at(i+1) << std::endl;
            std::cout << "pinv_J_i_P_i1: \n" << pinv_J_i_P_i1 << std::endl;
            std::cout << "x_dot_i: \n" << x_dot_i << std::endl;
            std::cout << "J_i: \n" << J_i << std::endl;
        }

        // Recursive formula
        q_vec.at(i) = q_vec.at(i+1) + pinv_J_i_P_i1 * (x_dot_i - J_i * q_vec.at(i+1));
    }

    // Returning the result
    q_sol = q_vec.at(0);
    return true;
}

bool reversePriorityManager::compute_proj_mats() {
    // Checking if there are any tasks in the set
    if (this->task_set_.empty()) {
        ROS_ERROR("There are no tasks in the set! Won't compute anything!");
        return false;
    }

    // Resizing the T matrices vector
    auto task_set_dim = this->task_set_.size();
    this->proj_mat_set_.resize(task_set_dim + 1);

    // Initial step of computation of T matrices
    auto n_rows = this->task_set_.at(task_set_dim - 1).get_task_jacobian().rows();
    auto n_cols = this->task_set_.at(task_set_dim - 1).get_task_jacobian().cols();

    Eigen::MatrixXd J_now = this->task_set_.at(task_set_dim - 1).get_task_jacobian();   // taking the last jacobian of the task set
    Eigen::MatrixXd J_aug = Eigen::MatrixXd::Zero(J_now.rows(), J_now.cols());
    this->proj_mat_set_.at(task_set_dim) = Eigen::MatrixXd::Identity(J_now.cols(), J_now.cols());

    if (DEBUG) {
        ROS_INFO_STREAM("The quantities for the " << int (task_set_dim) - 1 << "th Proj matrix computation are: ");
        std::cout << "J_now: \n" << J_now << std::endl;
        std::cout << "J_aug: \n" << J_aug << std::endl;
    }

    // Auxiliary Matrix for next loop
    Eigen::MatrixXd J_aux;

    // Recursion for the other T matrices
    for (int i = int (task_set_dim) - 2; i >= 0; i--) { // TODO : Test this (Inspired from new implementation in matlab)
        // Getting the new task jacobian
        Eigen::MatrixXd Jcurr = this->task_set_.at(i).get_task_jacobian();
        Eigen::MatrixXd Jprev = this->task_set_.at(i + 1).get_task_jacobian();

        // New number of rows and columns (of the new task jacobian)
        n_rows = Jprev.rows();
        n_cols = Jprev.cols();

        // Saving termporarily the old J_aug
        J_aux = J_aug;

        // Append the new task jacobian
        J_aug.resize(J_aux.rows() + Jprev.rows(), J_aux.cols());
        J_aug.block(0, 0, n_rows, n_cols) = Jprev;
        J_aug.block(int (n_rows), 0, J_aux.rows(), J_aux.cols()) = J_aux;

        // Clean jac and projection matrix
        Eigen::MatrixXd J_tilde = this->clean_jac(Jcurr.transpose(), J_aug.transpose()).transpose();
        Eigen::MatrixXd pinv_J_tilde = damped_pseudo_inv(J_tilde, this->lambda_max_, this->epsilon_);

        if (DEBUG) {
            ROS_INFO_STREAM("The quantities for the " << i << "th Proj matrix computation are: ");
            std::cout << "J_aug: \n" << J_aug << std::endl;
            std::cout << "J_tilde: \n" << J_tilde << std::endl;
        }

        this->proj_mat_set_.at(i + 1) = Eigen::MatrixXd::Identity(J_tilde.cols(), J_tilde.cols()) - pinv_J_tilde * J_tilde;
    }

    return true;
}

// Private Auxiliary Fuctions
Eigen::MatrixXd reversePriorityManager::clean_jac(Eigen::MatrixXd Jt, Eigen::MatrixXd Jrat) {

	//  CLEAN_JAC chooses the columns of Jrat that are not lin. dep. on Jt and returns a matrix that
	//  will have full column rank
	//  Returns: the mat with columns of Jrat that are not lin. dep. on Jt

    if (DEBUG) ROS_INFO("Entered CLEAN JAC!!!");

    // Getting the number of columns of Jrat
    auto dim_jrat = Jrat.cols();

    // At first T is Jt itself
    Eigen::MatrixXd T; T.resize(Jt.rows(), Jt.cols());
    T << Jt;
    bool first_it = true;
    Eigen::MatrixXd Tt;

	// Computing the rank of T
	Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(T);
	int prev_rank = lu_decomp.rank();

	// Now for all columns of Jrat checking if lin dep and adding
    for (int k = 0; k < dim_jrat; k++) {
    	// Appending the kth column of Jrat
    	T.conservativeResize(T.rows(), T.cols() + 1);
    	T.col(T.cols() - 1) = Jrat.col(k);

	    // Checking if rank increased
	    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(T);
	    int curr_rank = lu_decomp.rank();
	    if (curr_rank > prev_rank) {
	    	prev_rank = curr_rank;
	    	if (DEBUG) ROS_INFO("Rank increased in CLEAN JAC!!!");
            if (first_it) {
                Tt.resize(T.rows(),1);
                first_it = false;
            } else {
                Tt.conservativeResize(T.rows(),Tt.cols() + 1);
            }
            Tt.col(Tt.cols() - 1) = Jrat.col(k);
	    } else {
		    if (DEBUG) ROS_INFO("Rank did not increase in CLEAN JAC!!! Removing column!");
		    T.conservativeResize(T.rows(), T.cols() - 1);
		    if (DEBUG) ROS_INFO("In CLEAN JAC removed column!");
	    }
    }

    if (DEBUG) ROS_INFO("Exiting CLEAN JAC!!!");

    return Tt;
}
