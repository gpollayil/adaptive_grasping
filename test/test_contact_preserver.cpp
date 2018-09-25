#include <iostream>
#include <ros/ros.h>
#include "contactPreserver.h"

using namespace adaptive_grasping;

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Adaptive Grasping| -> Testing contactPreserver!"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "contact_preserver_test");

    ros::NodeHandle nh;

    // Creating needed variables
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(27, 1);

    // Creating object matricesCreator
    std::cout<<"Object contactPreserver being created!"<<std::endl;

    contactPreserver preserver(S);

    std::cout<<"Object contactPreserver created!"<<std::endl;

    // Trying to reset basic private variables
    S = Eigen::MatrixXd::Identity(33, 1);
    preserver.changeHandType(S);

    // Creating needed variables setting grasp state
    Eigen::MatrixXd J(3, 4);
    J << 1.5, 1.5, 0, 0, -1, 0, 0, 0, 1, 1, 0, 0;

    Eigen::MatrixXd G(3, 3);
    G << 1, 0, 0, 0, 1, -0.5, 0, 0, 1;

    Eigen::MatrixXd T(3, 3);
    T << 1, 0, 1.5, 0, 1, -1, 0, 0, 1;

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3, 3);

    // Setting grasp state
    preserver.setGraspState(J, G, T, H);

    // Creating needed variables for minimization
    Eigen::MatrixXd A_tilde = Eigen::MatrixXd::Identity(10, 10);
    A_tilde.block<3, 3>(7, 7) = 10 * Eigen::MatrixXd::Identity(3, 3);

    Eigen::VectorXd x_d(10);
    x_d << -1, 1, -1, -1, 1, -1, -1, 0, 0, 0;

    // Setting minimization parameters
    preserver.setMinimizationParams(x_d, A_tilde);

    // Performing minimization
    Eigen::VectorXd x_ref = preserver.performMinimization();

    ros::spin();

    return 0;
}
