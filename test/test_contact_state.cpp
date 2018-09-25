// For testing contactState class: we suppose the robot description is loaded
// and that on /touching_finger_topic a finger_id is published

#include <iostream>
#include <ros/ros.h>
#include "contactState.h"

using namespace adaptive_grasping;

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Adaptive Grasping| -> Testing contactState!"<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_contact_state");

    ros::NodeHandle nh;

    // Creating needed topic and maps for contact_state constructor
    std::string topic_name_test = "/touching_finger_topic";
    std::map<int, std::string> link_names_map_test;
    std::map<std::string, std::string> params_map_test;

    // Filling up the maps
    link_names_map_test[1] = "right_hand_thumb_distal_link";
    link_names_map_test[2] = "right_hand_index_distal_link";
    link_names_map_test[3] = "right_hand_middle_distal_link";
    link_names_map_test[4] = "right_hand_ring_distal_link";
    link_names_map_test[5] = "right_hand_little_distal_link";
    params_map_test["world_name"] = "world";
    params_map_test["palm_name"] = "right_hand_palm_link";
    params_map_test["ee_name"] = "right_hand_ee_link";

    // Creating an object contact_state
    std::cout<<"Starting to create Object contactState!"<<std::endl;
    contactState contact_state_obj(topic_name_test, link_names_map_test, params_map_test);
    std::cout<<"Object contactState created successfully!"<<std::endl;

    // Creating two maps which will be couted
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map_test;
    std::map<int, sensor_msgs::JointState> joints_map_test;

    while(ros::ok()){
      ros::spinOnce();

      // Reading the values of the contact_state_obj
      contact_state_obj.readValues(contacts_map_test, joints_map_test);
      
    }

    return 0;
}
