#include <iostream>
#include <ros/ros.h>
#include "matricesCreator.h"

using namespace adaptive_grasping;

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Adaptive Grasping| -> Testing matricesCreator!"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "matrices_creator_test");

    ros::NodeHandle nh;

    // Creating needed variables
    Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);

    Eigen::MatrixXd H_i;
    H_i << I_3, O_3, O_3, I_3;

    std::string world_frame_name = "/world";
    std::string palm_frame_name = "/right_hand_palm_link";
    std::vector<int> joint_numbers = {5, 7, 7, 7, 7};

    matricesCreator creator(H_i, world_frame_name, palm_frame_name, joint_numbers);

    std::cout<<"Object matricesCreator created!"<<std::endl;

    ros::spin();

    return 0;
}
