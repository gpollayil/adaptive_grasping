// For testing the three contact_state, matrices_creator and contact_preserver
// classes

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "contactState.h"
#include "matricesCreator.h"
#include "contactPreserver.h"

#define DEBUG   0

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
  params_map_test["palm_name"] = "right_arm_7_link";            // THIS IS NEEDED BECAUSE KUKA CONTROLLERS CANNOT HAVE AS TIP LINK A SOFTHAND'S ONE
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
  affine.pretranslate(Eigen::Vector3d(0, 0, -0.15));
  std::cout << "Obj translation is: " << affine.translation() << std::endl;

  Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);

  // Soft finger contact constraint matrix (comment out the 3rd and 4th line for fully constrianted)
  Eigen::MatrixXd H_i(6, 6);
  H_i << I_3, O_3, O_3, I_3;
  // H_i(3, 3) = 0;
  // H_i(4, 4) = 0;
  // H_i(5, 5) = 0;

  std::string world_frame_name = "world";
  std::string palm_frame_name = "right_arm_7_link";
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
  // Eigen::MatrixXd S = Eigen::MatrixXd::Identity(33, 33);
  
  // THIS IS THE SYNERGY MATRIX OF PISA IIT SH OCADO VERSION - AS IN URDF
  Eigen::MatrixXd S(33, 1);
  S << 2.042, 1.021, 1.021, 0.785, 0.785, -0.2, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.2, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.4, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785;

  // Creating needed variables for minimization
  // Eigen::MatrixXd A_tilde = Eigen::MatrixXd::Identity(45, 45);
  // A_tilde.block<6, 6>(39, 39) = 10 * Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd A_tilde = Eigen::MatrixXd::Identity(13, 13);
  A_tilde.block<6, 6>(7, 7) = 10000 * Eigen::MatrixXd::Identity(6, 6);

  // Eigen::VectorXd x_d(45);
  Eigen::VectorXd x_d(13);
  Eigen::VectorXd x_ref;

  // Creating object matricesCreator
  std::cout<<"Object contactPreserver being created!"<<std::endl;
  contactPreserver preserver(S);
  std::cout<<"Object contactPreserver created!"<<std::endl;

  ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>(
    "right_arm/twist_controller/command", 1);
  ros::Publisher pub_cmd_hand = nh.advertise<std_msgs::Float64>(
    "right_hand/velocity_controller/command", 1);

  // Setting the ROS rate for the while as it is in the controller
  ros::Rate rate(1000);

  // For getting the period of the while loop
  ros::Time initial_time; ros::Duration duration_loop;

  // Command vars
  geometry_msgs::Twist cmd_twist;
  std_msgs::Float64 cmd_syn;

  // For publishing null commands, creating null twist and synergy
  geometry_msgs::Twist cmd_twist_null;
  cmd_twist_null.linear.x = 0; cmd_twist_null.angular.x = 0;
  cmd_twist_null.linear.y = 0; cmd_twist_null.angular.y = 0;
  cmd_twist_null.linear.z = 0; cmd_twist_null.angular.z = 0;

  std_msgs::Float64 cmd_syn_null;
  cmd_syn_null.data = float (0.0);

  // Trial twist
  // geometry_msgs::Twist trial_twist;
  // trial_twist.linear.x = 0; trial_twist.angular.x = 0;
  // trial_twist.linear.y = 0; trial_twist.angular.y = 0;
  // trial_twist.linear.z = 0; trial_twist.angular.z = 0;

  while(ros::ok()){
    // Publishing null commands for avoiding the repetition of old refs while computing
    pub_cmd_hand.publish(cmd_syn_null);
    pub_cmd.publish(cmd_twist_null);

    // Getting initial time
    initial_time = ros::Time::now();

    // Spinning once to process callbacks
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
    if(DEBUG) std::cout << "The created matrices are: " << std::endl;
    if(DEBUG) std::cout << "J = " << "\n";
    if(DEBUG) std::cout << read_J << "\n";
    if(DEBUG) std::cout << "G = " << "\n";
    if(DEBUG) std::cout << read_G << "\n";
    if(DEBUG) std::cout << "T = " << "\n";
    if(DEBUG) std::cout << read_T << "\n";
    if(DEBUG) std::cout << "H = " << "\n";
    if(DEBUG) std::cout << read_H << "\n";

    // THE FOLLOWING WILL BE DONE ONLY IF THE NEEDED MATRICES ARE != 0
    x_ref = Eigen::VectorXd::Zero(x_d.size());

    if(read_J.innerSize() > 0 && read_G.innerSize() > 0 &&
      read_T.innerSize() > 0 && read_H.innerSize() > 0){
        // Setting grasp state
        preserver.setGraspState(read_J, read_G, read_T, read_H);

        // Setting minimization parameters
        // x_d = Eigen::VectorXd::Zero(45);
        // x_d(35) = -0.05;
        x_d = Eigen::VectorXd::Zero(13);
        x_d(0) = 0.1; 
        x_d(3) = -0.01; x_d(5) = -0.1;
        preserver.setMinimizationParams(x_d, A_tilde);

        // Performing minimization
        x_ref = preserver.performMinimization();
    }

    // Print out all variables in contactPreserver
    // preserver.printAll();

    // Print out the resulting motion
    std::cout << "Resulting reference motion x_ref is:" << std::endl;
    std::cout << x_ref << std::endl;

    double scaling = 1.0;

    // Converting to geometry_msgs the twist of the palm and publishing
    cmd_twist.linear.x = scaling*x_ref(1); cmd_twist.angular.x = scaling*x_ref(4);
    cmd_twist.linear.y = scaling*x_ref(2); cmd_twist.angular.y = scaling*x_ref(5);
    cmd_twist.linear.z = scaling*x_ref(3); cmd_twist.angular.z = scaling*x_ref(6);

    cmd_syn.data = float (scaling*x_ref(0));

    // Getting computation time and couting
    duration_loop = ros::Time::now() - initial_time;
    std::cout << "The computation time was " << duration_loop.toNSec() << "s." << std::endl;

    pub_cmd_hand.publish(cmd_syn);
    pub_cmd.publish(cmd_twist);

    // Sleeping for some time in order to execute the command before cancelling
    ros::Duration(0.005).sleep();

    // Rate
    rate.sleep();
  }
}
