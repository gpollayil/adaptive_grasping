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
reversePriorityManager::reversePriorityManager(int dim_config_space, double lambda_max, double epsilon) {
    // Setting the dimension of the configuration space of the tasks and the RP constants
    this->dim_config_space_ = dim_config_space;
    this->lambda_max_ = lambda_max;
    this->epsilon_ = epsilon;
}

// Overloaded Constructor
reversePriorityManager::reversePriorityManager(int dim_config_space, double lambda_max, double epsilon, std::vector<basicTask> starting_task_set) {
    // Setting the dimension of the configuration space of the tasks and the RP constants
    this->dim_config_space_ = dim_config_space;
    this->lambda_max_ = lambda_max;
    this->epsilon_ = epsilon;

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

bool reversePriorityManager::solve_inv_kin(Eigen::VectorXd &q_sol) {
    // Initializing the result vector
    Eigen::VectorXd q_res; q_res.resize(this->dim_config_space_); q_res.setZero();

    // Computing all the T matrices
    if (this->compute_T_mats()) {
        ROS_ERROR("Could not compute any task! Won't solve anything!");
        q_sol = q_res;
        return false;
    }

    // Size of task set
    auto task_set_dim = this->task_set_.size();

    // Creating vector of RP recursion and its last element is the initial guess
    std::vector<Eigen::VectorXd> q_vec;
    q_vec.resize(task_set_dim);
    q_vec.at(task_set_dim) = Eigen::VectorXd::Zero(this->dim_config_space_);

    // The RP recursion (ref. paper)
    for (int i = int (task_set_dim) - 1; i >= 0; i--) {
        // DO THIS!!!!!!
        // DO THIS!!!!!!
        // DO THIS!!!!!!
        // DO THIS!!!!!!
        // DO THIS!!!!!!
    }
}

bool reversePriorityManager::compute_T_mats() {
    // Checking if there are any tasks in the set
    if (this->task_set_.empty()) {
        ROS_ERROR("There are no tasks in the set! Won't compute anything!");
        return false;
    }

    // Resizing the T matrices vector
    auto task_set_dim = this->task_set_.size();
    this->t_proj_mat_set_.resize(task_set_dim - 1);

    // Initial step of computation of T matrices
    auto n_rows = this->task_set_.at(task_set_dim).get_task_jacobian().rows();
    auto n_cols = this->task_set_.at(task_set_dim).get_task_jacobian().cols();

    Eigen::MatrixXd J_aug = this->task_set_.at(task_set_dim).get_task_jacobian();   // taking the last jacobian of the task set
    Eigen::MatrixXd J_aug_pinv = this->damped_pseudo_inv(J_aug, this->lambda_max_, this->epsilon_);
    Eigen::MatrixXd T_aux = this->rank_update(J_aug, J_aug_pinv);
    this->t_proj_mat_set_.at(task_set_dim - 1) = T_aux;

    // Recursion for the other T matrices
    for (int i = int (task_set_dim) - 2; i >= 0; i--) { // TODO : Test this (I feel this is not correct!)
        // Getting the new task jacobian
        Eigen::MatrixXd Jcurr = this->task_set_.at(i).get_task_jacobian();

        // New number of rows and columns (of the new task jacobian)
        n_rows = Jcurr.rows();
        n_cols = Jcurr.cols();

        // Append the new task jacobian
        J_aug.resize(J_aug.rows() + Jcurr.rows(), J_aug.cols());
        J_aug.block(0, 0, n_rows, n_cols) = Jcurr;
        J_aug.block(n_rows, 0, J_aug.rows(), J_aug.cols()) = J_aug;

        // Temporary variables
        Eigen::MatrixXd Jpinv_tmp;
        Eigen::MatrixXd Aux_mat;

        // Pseudo inversion and rank update
        Jpinv_tmp = this->damped_pseudo_inv(J_aug, this->lambda_max_, this->epsilon_);
        Aux_mat = this->rank_update(J_aug, Jpinv_tmp);

        this->t_proj_mat_set_.at(i) = Aux_mat;
    }
}

// Private Auxiliary Fuctions
Eigen::MatrixXd reversePriorityManager::damped_pseudo_inv(Eigen::MatrixXd input_mat, double damping_coeff, double epsilon) {
    // Computing the singular values
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();

    // Checking if the smallest sing val is really small
    double lambda_sq = 0.0;
    double min_sing_val = sing_vals(sing_vals.size() - 1);
    if (min_sing_val < epsilon) {
        lambda_sq = (1 - pow((min_sing_val / epsilon), 2)) * pow(damping_coeff, 2);
    }

    // Changing the diagonal sv matrix
    Eigen::MatrixXd S = input_mat; S.setZero();
    for (int i = 0; i < sing_vals.size(); i++) {
        S(i, i) = (sing_vals(i)) / (pow(sing_vals(i), 2) + lambda_sq);
    }

    // Return the svd based damped pseudoinverse
    return Eigen::MatrixXd(svd.matrixV() * S.transpose() * svd.matrixU().transpose());
}

Eigen::MatrixXd reversePriorityManager::rank_update(Eigen::MatrixXd J, Eigen::MatrixXd Jra_pinv) {
    // Getting the rank of J and the number of columns of Jra_pinv
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(J);
    auto rank_j = lu_decomp.rank();
    auto dim_jra = Jra_pinv.cols();

    // At first T is the first column of Jra_pinv
    Eigen::MatrixXd T; T.resize(Jra_pinv.rows(), 1);
    T << Jra_pinv.col(0);

    // The rank update procedure (ref Matlab implementation)
    int i = 1; int j = 1;
    while (i <= rank_j - 1) {
        while (j <= dim_jra - 1) {
            // Adding the ith column to T
            T.conservativeResize(T.rows(), T.cols() + 1);
            T.col(T.cols() - 1) = Jra_pinv.col(j);
            j++;

            // Looking if rank changed
            Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(T);
            auto rank_t = lu_decomp.rank();
            if (rank_t == i + 1) { // rank changed because new column is lin. indep.
                i++; break;
            } else { // rank didn't change so removing column
                T.conservativeResize(T.rows(), T.cols() - 1);
            }
        }
    }

    // Checking for bounty of result
    if (T.cols() < rank_j) ROS_ERROR_STREAM("The rank update procedure did not lead to good results!");

    return T;
}