// For testing the three contact_state, matrices_creator and contact_preserver
// classes

#include <iostream>
#include <ros/ros.h>
#include "contactState.h"
#include "matricesCreator.h"
#include "contactPreserver.h"

using namespace adaptive_grasping;

int main(int argc, char **argv){

  std::cout<<std::endl;
  std::cout<<"|Adaptive Grasping| -> Testing contactState, matricesCreator "
    "and contactPreserver!" <<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "test_state_creator_preserver");
  ros::NodeHandle nh;

  /*
      FOR CONTACT_STATE
  */

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

  /*
      FOR MATRICES_CREATOR
  */

  // Creating needed variables
  tf::TransformListener tf_listener;
  tf::StampedTransform stamped_transform;

  // Echoing the object pose and converting to Affine3d
  ROS_INFO("Waiting to get the palm tf!!!");
  try {
  tf_listener.waitForTransform("/world", "/right_hand_palm_link",
    ros::Time(0), ros::Duration(20.0) );
  tf_listener.lookupTransform("/world", "/right_hand_palm_link",
    ros::Time(0), stamped_transform);
  } catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  Eigen::Affine3d affine;
  tf::Transform transform(stamped_transform.getRotation(),
    stamped_transform.getOrigin());
  tf::transformTFToEigen(transform, affine);
  std::cout << "Palm translation is: " << affine.translation() << std::endl;
  affine.pretranslate(Eigen::Vector3d(0, 0, 0.15));
  std::cout << "Obj translation is: " << affine.translation() << std::endl;

  Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);

  Eigen::MatrixXd H_i(6, 6);
  H_i << I_3, O_3, O_3, I_3;

  std::string world_frame_name = "world";
  std::string palm_frame_name = "right_hand_palm_link";
  std::vector<int> joint_numbers = {5, 7, 7, 7, 7};

  // For reading and couting
  Eigen::MatrixXd read_J; Eigen::MatrixXd read_G;
  Eigen::MatrixXd read_T; Eigen::MatrixXd read_H;

  // Creating object matricesCreator
  std::cout<<"Object matricesCreator being created!"<<std::endl;
  matricesCreator creator(H_i, world_frame_name, palm_frame_name, joint_numbers);
  std::cout<<"Object matricesCreator created!"<<std::endl;

  /*
      FOR CONTACT_PRESERVER
  */

  // Creating needed variables
  Eigen::MatrixXd S = Eigen::MatrixXd::Identity(33, 33);

  // Creating needed variables for minimization
  Eigen::MatrixXd A_tilde = Eigen::MatrixXd::Identity(45, 45);
  A_tilde.block<6, 6>(39, 39) = 10 * Eigen::MatrixXd::Identity(6, 6);

  Eigen::VectorXd x_d(45);
  Eigen::VectorXd x_ref;

  // Creating object matricesCreator
  std::cout<<"Object contactPreserver being created!"<<std::endl;
  contactPreserver preserver(S);
  std::cout<<"Object contactPreserver created!"<<std::endl;

  while(ros::ok()){
    ros::spinOnce();

    // Reading the values of the contact_state_obj
    contact_state_obj.readValues(contacts_map_test, joints_map_test);

    // Setting the contacts_map and joints_map in creator and computing matrices
    creator.setContactsMap(contacts_map_test);
    creator.setJointsMap(joints_map_test);
    creator.setObjectPose(affine);
    creator.computeAllMatrices();

    // Reading and couting the matrices
    creator.readAllMatrices(read_J, read_G, read_T, read_H);
    std::cout << "The created matrices are: " << std::endl;
    std::cout << "J = " << "\n";
    std::cout << read_J << "\n";
    std::cout << "G = " << "\n";
    std::cout << read_G << "\n";
    std::cout << "T = " << "\n";
    std::cout << read_T << "\n";
    std::cout << "H = " << "\n";
    std::cout << read_H << "\n";

    // Setting grasp state
    preserver.setGraspState(read_J, read_G, read_T, read_H);
    
    // Setting minimization parameters
    x_d = Eigen::VectorXd::Zero(45);
    x_d(35) = -0.05;
    preserver.setMinimizationParams(x_d, A_tilde);

    // Performing minimization
    x_ref = preserver.performMinimization();

    // Print out all variables in contactPreserver
    preserver.printAll();

    // Print out the resulting motion
    std::cout << "Resulting reference motion x_ref is:" << std::endl;
    std::cout << x_ref << std::endl;
  }
}
