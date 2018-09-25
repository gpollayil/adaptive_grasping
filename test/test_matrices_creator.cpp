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

    Eigen::MatrixXd H_i(6, 6);
    H_i << I_3, O_3, O_3, I_3;

    std::string world_frame_name = "/world";
    std::string palm_frame_name = "/right_hand_palm_link";
    std::vector<int> joint_numbers = {5, 7, 7, 7, 7};

    // Creating object matricesCreator
    std::cout<<"Object matricesCreator being created!"<<std::endl;

    matricesCreator creator(H_i, world_frame_name, palm_frame_name, joint_numbers);

    std::cout<<"Object matricesCreator created!"<<std::endl;

    // Trying to reset basic private variables
    creator.changeHandType(H_i);
    creator.changeFrameNames(world_frame_name, palm_frame_name);

    // Creating needed variables for a contacts map
    std::string c1_frame_name = "/right_hand_thumb_distal_link";
    std::string c2_frame_name = "/right_hand_middle_distal_link";

    Eigen::Affine3d w1(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));
    Eigen::Affine3d pc1(Eigen::Translation3d(Eigen::Vector3d(1,0,2)));

    Eigen::Affine3d w2 =
      Eigen::Affine3d(Eigen::AngleAxisd(0.1, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d pc2 =
      Eigen::Affine3d(Eigen::AngleAxisd(0.2, Eigen::Vector3d(0, 1, 0)));

    std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d> contacts_tuple1 =
      std::make_tuple(c1_frame_name, w1, pc1);
    std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d> contacts_tuple2 =
      std::make_tuple(c2_frame_name, w2, pc2);

    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map;

    contacts_map[1] = contacts_tuple1;
    contacts_map[3] = contacts_tuple2;

    // Setting the contacts map in creator
    creator.setContactsMap(contacts_map);

    // Trying to compute all matrices
    creator.computeAllMatrices();

    ros::spin();

    return 0;
}
