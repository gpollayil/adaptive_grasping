/* For testing reversePriorityManager class */

// Basic Includes
#include <iostream>
#include <ros/ros.h>


#include "basicTask.h"
#include "reversePriorityManager.h"

using namespace adaptive_grasping;

int main(int argc, char **argv) {

    // Starting the test node
    std::cout<<std::endl;
    std::cout<<"|Adaptive Grasping| -> Testing Reverse Priority!"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_reverse_priority");
    ros::NodeHandle nh;

    // Creating the objects
    std::vector<basicTask> task_vec;
    task_vec.clear();
    reversePriorityManager rp_manager(10, 1000, 0.1);

    // Temporary variables
    basicTask tmp_task;
    Eigen::VectorXd tmp_x_dot;
    Eigen::MatrixXd tmp_jac;
    int tmp_priority;

    // First task
    tmp_x_dot.resize(7);
    tmp_x_dot << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    tmp_jac.resize(7, 10);
    tmp_jac << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            1.5, 1.5, 0.0, 0.0, 1.0, 0.0, 1.5, -1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, -1.0, 0.5,
            1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0;
    tmp_priority = 1;

    tmp_task.set_task_x_dot(tmp_x_dot);
    tmp_task.set_task_jacobian(tmp_jac);
    tmp_task.set_task_priority(tmp_priority);

    task_vec.push_back(tmp_task);

    // Second task
    tmp_x_dot.resize(7);
    tmp_x_dot << -1.0, 1.0, -1.0, -1.0, 1.0, -1.0, -1.0;
    tmp_jac.resize(7, 10);
    tmp_jac << Eigen::MatrixXd::Identity(7, 10);
    tmp_priority = 2;

    tmp_task.set_task_x_dot(tmp_x_dot);
    tmp_task.set_task_jacobian(tmp_jac);
    tmp_task.set_task_priority(tmp_priority);

    task_vec.push_back(tmp_task);

    // Filling up RP Manager
    rp_manager.clear_set();
    rp_manager.insert_tasks(task_vec);

    // Print out of the task set
    rp_manager.print_set();

    // Computing and couting the RP solution
    Eigen::VectorXd x_ref;
    if(rp_manager.solve_inv_kin(x_ref)) ROS_INFO_STREAM("The RP Solution is \n" << x_ref);
    else ROS_ERROR("RP Manager could not find solution!");

    ROS_INFO("Exiting RP Test File");
    return 0;
}
