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
    Eigen::MatrixXd S_i = Eigen::MatrixXd::Identity(27, 1);

    // Creating object matricesCreator
    std::cout<<"Object contactPreserver being created!"<<std::endl;

    contactPreserver preserver(S_i);

    std::cout<<"Object contactPreserver created!"<<std::endl;

    // Trying to reset basic private variables
    Eigen::MatrixXd S_i = Eigen::MatrixXd::Identity(33, 1);
    preserver.changeHandType(S_i);

    // Creating needed variables for a contacts map


    ros::spin();

    return 0;
}
