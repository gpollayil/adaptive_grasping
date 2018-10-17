// For testing the three contact_state, matrices_creator and contact_preserver
// classes

#include <iostream>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include "contactState.h"
#include "matricesCreator.h"
#include "contactPreserver.h"
#include <utils/pseudo_inversion.h>
#include <ros/subscribe_options.h>

// For RViz Visualization
#include <visualization_msgs/Marker.h>

// Service Includes
#include "adaptive_grasping/velCommand.h"

// Action Client
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define DEBUG   0

using namespace adaptive_grasping;

// GLOBAL VARIABLES
sensor_msgs::JointState::ConstPtr full_joint_state;	  // a msg where the subscriber will save the joint states
KDL::Tree robot_kin_tree;
KDL::Chain robot_kin_chain;

/**********************************************************************************************
 JOINTS EXTRACT
**********************************************************************************************/
Eigen::VectorXd jointsExtract(){
  // Creating the vector to be returned
  Eigen::VectorXd vector_js(8);

  // Finding the states of arm and hand and pushing back
  int index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_arm_a1_joint") - full_joint_state->name.begin();
	vector_js(0) = full_joint_state->position[index];

  index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_arm_a2_joint") - full_joint_state->name.begin();
	vector_js(1) = full_joint_state->position[index];

  index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_arm_e1_joint") - full_joint_state->name.begin();
	vector_js(2) = full_joint_state->position[index];

  index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_arm_a3_joint") - full_joint_state->name.begin();
	vector_js(3) = full_joint_state->position[index];

  index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_arm_a4_joint") - full_joint_state->name.begin();
	vector_js(4) = full_joint_state->position[index];

  index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_arm_a5_joint") - full_joint_state->name.begin();
	vector_js(5) = full_joint_state->position[index];

  index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_arm_a6_joint") - full_joint_state->name.begin();
	vector_js(6) = full_joint_state->position[index];

  index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_hand_synergy_joint") - full_joint_state->name.begin();
	vector_js(7) = full_joint_state->position[index];

  return vector_js;
}

/**********************************************************************************************
 SUBSCRIBER CALLBACK
**********************************************************************************************/
void getJointStates(const sensor_msgs::JointState::ConstPtr &msg){
	// Storing the message into another global message variable
	ROS_DEBUG_STREAM("GOT JOINTSTATE MSG: STARTING TO SAVE!");
	full_joint_state = msg;
	ROS_DEBUG_STREAM("SAVED JOINTSTATE MSG!");
}

/**********************************************************************************************
 COMPUTE SYNERGY MATRIX
**********************************************************************************************/
Eigen::MatrixXd computeSynergyMatrix(){
	// Using the full joint state to compute the ratios
  Eigen::MatrixXd Syn(33, 1);

  // Copying the values of the joints
  int index = find (full_joint_state->name.begin(),full_joint_state->name.end(), "right_hand_thumb_abd_joint") - full_joint_state->name.begin();
	for(int j = 0; j < 33; j++){
    Syn(j) = full_joint_state->position[index + j];
  }

  // Dividing by synergy value to find the Matrix
  Syn = Syn / full_joint_state->position[index - 1];

  return Syn;
}

/**********************************************************************************************
 SET KDL
**********************************************************************************************/
bool setKDL(ros::NodeHandle node){
  // Load robot description from ROS parameter server
  std::string robot_description_string;
  node.param("robot_description", robot_description_string, std::string());

  // Get kinematic tree from robot description
  if (!kdl_parser::treeFromString(robot_description_string, robot_kin_tree)){
    ROS_ERROR("Failed to get robot kinematic tree!");
    return false;
  }

  // Getting chain
  robot_kin_tree.getChain("world", "right_arm_7_link", robot_kin_chain);
}

/**********************************************************************************************
 MAIN
**********************************************************************************************/

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

  sleep(2.0);

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
  affine.pretranslate(Eigen::Vector3d(0.08, 0, -0.08));
  std::cout << "Obj translation is: " << affine.translation() << std::endl;
  Eigen::Vector3d obj_trans = affine.translation();

  // Publishing the object to RViz
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("object_marker", 1);
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  visualization_msgs::Marker obj_marker;
  obj_marker.header.frame_id = "/world";
  obj_marker.header.stamp = ros::Time::now();
  obj_marker.ns = "adaptive_grasping";
  obj_marker.id = 0;
  obj_marker.type = shape;
  obj_marker.action = visualization_msgs::Marker::ADD;
  obj_marker.pose.position.x = obj_trans(0); obj_marker.pose.position.y = obj_trans(1);
  obj_marker.pose.position.z = obj_trans(2);
  obj_marker.scale.x = 0.05; obj_marker.scale.y = 0.05; obj_marker.scale.z = 0.05;
  obj_marker.color.r = 0.0f; obj_marker.color.g = 1.0f; obj_marker.color.b = 0.0f; obj_marker.color.a = 1.0f;
  obj_marker.lifetime = ros::Duration(0);
  marker_pub.publish(obj_marker);

  Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);

  // Fully constrained contact matrix
  // Eigen::MatrixXd H_i(6, 6);
  // H_i << I_3, O_3, O_3, I_3;

  // Position constrained contact matrix
  Eigen::MatrixXd H_i(3, 6);
  H_i << I_3, O_3;


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

  // The subscriber for saving joint states
	// ros::SubscribeOptions joint_state_so = ros::SubscribeOptions::create<sensor_msgs::JointState>("joint_states",
	// 	1, getJointStates, ros::VoidPtr(), nh.getCallbackQueue());
	// ros::Subscriber js_sub = nh.subscribe(joint_state_so);
  ros::Subscriber js_sub = nh.subscribe("joint_states", 1, getJointStates);
  ROS_WARN_STREAM("The subsciber subscribed to " << js_sub.getTopic() << ".");

  // Waiting for a message in joint states
  full_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);

  // Spinning once to process callbacks
  ros::spinOnce();

  // Creating needed variables
  // Eigen::MatrixXd S = Eigen::MatrixXd::Identity(33, 33);

  // THIS IS THE SYNERGY MATRIX OF PISA IIT SH OCADO VERSION - AS IN URDF
  Eigen::MatrixXd S(33, 1);
  S = computeSynergyMatrix();
  // S << 2.042, 1.021, 1.021, 0.785, 0.785, -0.2, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.2, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785, 0.4, 0.785, 0.785, 0.785, 0.785, 0.785, 0.785;

  // Synergy matrix not constant????!!!!

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

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client("/right_arm/joint_trajectory_controller/follow_joint_trajectory/", true);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> hand_client("/right_hand/joint_trajectory_controller/follow_joint_trajectory/", true);

  // Creating follow_joint_traj commands
  control_msgs::FollowJointTrajectoryGoal arm_goalmsg;
  control_msgs::FollowJointTrajectoryGoal hand_goalmsg;

  // Creating trajectory messages
  trajectory_msgs::JointTrajectoryPoint tmp_point;

  trajectory_msgs::JointTrajectory arm_traj;
  arm_traj.joint_names.push_back("right_arm_a1_joint"); arm_traj.joint_names.push_back("right_arm_a2_joint");
  arm_traj.joint_names.push_back("right_arm_e1_joint"); arm_traj.joint_names.push_back("right_arm_a3_joint");
  arm_traj.joint_names.push_back("right_arm_a4_joint"); arm_traj.joint_names.push_back("right_arm_a5_joint");
  arm_traj.joint_names.push_back("right_arm_a6_joint");

  trajectory_msgs::JointTrajectory hand_traj;
  hand_traj.joint_names.push_back("right_hand_synergy_joint");

  // Setting KDL and solver
  if(!setKDL(nh)) ROS_ERROR_STREAM("Could not initialize KDL!");
  KDL::Jacobian arm_jac; arm_jac.resize(robot_kin_chain.getNrOfJoints());
  KDL::ChainJntToJacSolver jacobian_solver(robot_kin_chain);
  KDL::JntArray arm_js; arm_js.resize(robot_kin_chain.getNrOfJoints());
  Eigen::MatrixXd arm_jac_pinv;

  // The current joints state and required arm velocity
  Eigen::VectorXd curr_js;              // Used to get both js of arm and hand using jointsExtract()
  Eigen::VectorXd req_jvel;             // Required arm joint vel to follow twist
  ros::Duration dt(0.01, 0);
  ros::Duration dt_cont(0.01, 0);

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

  // Creating a service client for robot commander
  ros::ServiceClient client_rc = nh.serviceClient<adaptive_grasping::velCommand>("rc_jt_service");

  // Waiting for a message in joint states
  full_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);

  // Spinning once to process callbacks
  ros::spinOnce();

  // Getting the current needed joints
  Eigen::VectorXd joints_trial = jointsExtract();
  double res = 0.01;

  // TRIAL
    // arm_traj.header.stamp = ros::Time::now() + ros::Duration(0, 150000000);
    // tmp_point.positions.clear();
    // arm_traj.points.clear();
    // tmp_point.positions.push_back(joints_trial(0) + res);
    // tmp_point.positions.push_back(joints_trial(1) + res);
    // tmp_point.positions.push_back(joints_trial(2) + res);
    // tmp_point.positions.push_back(joints_trial(3) + res);
    // tmp_point.positions.push_back(joints_trial(4) + res);
    // tmp_point.positions.push_back(joints_trial(5) + res);
    // tmp_point.positions.push_back(joints_trial(6) + res);
    // tmp_point.time_from_start = ros::Duration(0.9, 0);
    // arm_traj.points.push_back(tmp_point);
    // tmp_point.positions.clear();
    // tmp_point.positions.push_back(joints_trial(0) + 2*res);
    // tmp_point.positions.push_back(joints_trial(1) + 2*res);
    // tmp_point.positions.push_back(joints_trial(2) + 2*res);
    // tmp_point.positions.push_back(joints_trial(3) + 2*res);
    // tmp_point.positions.push_back(joints_trial(4) + 2*res);
    // tmp_point.positions.push_back(joints_trial(5) + 2*res);
    // tmp_point.positions.push_back(joints_trial(6) + 2*res);
    // tmp_point.time_from_start = ros::Duration(1.8, 0);
    // arm_traj.points.push_back(tmp_point);
    // // tmp_point.positions.clear();
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.time_from_start = ros::Duration(3, 0);
    // // arm_traj.points.push_back(tmp_point);

    // hand_traj.header.stamp = ros::Time::now() + ros::Duration(0, 150000000);
    // tmp_point.positions.clear();
    // hand_traj.points.clear();
    // tmp_point.positions.push_back(joints_trial(7) + res);
    // tmp_point.time_from_start = ros::Duration(0.9, 0);
    // hand_traj.points.push_back(tmp_point);
    // tmp_point.positions.clear();
    // tmp_point.positions.push_back(joints_trial(7) + 2*res);
    // tmp_point.time_from_start = ros::Duration(1.8, 0);
    // hand_traj.points.push_back(tmp_point);
    // // tmp_point.positions.clear();
    // // tmp_point.positions.push_back(0.2);
    // // tmp_point.time_from_start = ros::Duration(3, 0);
    // // hand_traj.points.push_back(tmp_point);

    // // Setting goal msgs
    // arm_goalmsg.trajectory = arm_traj;
    // hand_goalmsg.trajectory = hand_traj;

    // // Publishing to joint traj controllers
    // arm_client.sendGoal(arm_goalmsg);
    // hand_client.sendGoal(hand_goalmsg);

    // if(!arm_client.waitForResult(ros::Duration(10, 0))){
  	// 	ROS_WARN_STREAM("The arm is taking too much time to close! \n");
  	// }
    // if(!hand_client.waitForResult(ros::Duration(10, 0))){
  	// 	ROS_WARN_STREAM("The hand is taking too much time to close! \n");
  	// }

    // sleep(0.1);

    // END TRIAL


  while(ros::ok()){
    // Spinning once to process callbacks
    ros::spinOnce();

    // Publishing the marker
    marker_pub.publish(obj_marker);

    // Getting initial time
    initial_time = ros::Time::now();

    // Reading the values of the contact_state_obj
    contact_state_obj.readValues(contacts_map_test, joints_map_test);

    // Setting the contacts_map and joints_map in creator and computing matrices
    S = computeSynergyMatrix();
    ROS_DEBUG_STREAM("The synergy matrix : \n" << S << ".");
    creator.setContactsMap(contacts_map_test);
    creator.setJointsMap(joints_map_test);
    creator.setObjectPose(affine);
    creator.computeAllMatrices();
    preserver.changeHandType(S);

    // Reading and couting the matrices
    creator.readAllMatrices(read_J, read_G, read_T, read_H);
    ROS_DEBUG_STREAM("The created matrices are: ");
    ROS_DEBUG_STREAM("\nJ = " << "\n" << read_J << "\n");
    ROS_INFO_STREAM("\nG = " << "\n" << read_G << "\n");
    ROS_INFO_STREAM("\nT = " << "\n" << read_T << "\n");
    ROS_DEBUG_STREAM("\nH = " << "\n" << read_H << "\n");

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

        ROS_DEBUG_STREAM("Performed Minimization!!!");
    }

    // Scaling all
    double scale = 0.08;
    x_ref = scale * x_ref;

    // Scaling only palm twist
    double lin_scaling = 1;
    x_ref(1) = lin_scaling*x_ref(1); x_ref(4) = lin_scaling*x_ref(4);
    x_ref(2) = lin_scaling*x_ref(2); x_ref(5) = lin_scaling*x_ref(5);
    x_ref(3) = lin_scaling*x_ref(3); x_ref(6) = lin_scaling*x_ref(6);

    std::cout << "x_ref = " << "\n";
    std::cout << x_ref << "\n";

    // Filling and calling robot commander service
    // double diff_scale = 1.0;
    // adaptive_grasping::velCommand command;
    // command.request.x_ref.push_back(diff_scale * x_ref(2));
    // command.request.x_ref.push_back(diff_scale * x_ref(1));
    // command.request.x_ref.push_back(diff_scale * x_ref(3));
    // command.request.x_ref.push_back(diff_scale * x_ref(4));
    // command.request.x_ref.push_back(diff_scale * x_ref(5));
    // command.request.x_ref.push_back(diff_scale * x_ref(6));
    // command.request.x_ref.push_back(x_ref(0));

    // if(client_rc.call(command)){
    //   ROS_INFO_STREAM("Success!!!!");
    // } else {
    //   ROS_INFO_STREAM("Failed!!!!");
    // }

    // Getting current joint states
    // curr_js = jointsExtract();

    // // Getting the current jacobian and its pseudoinv
    // arm_js.data = curr_js.head(7);
    // jacobian_solver.JntToJac(arm_js, arm_jac);
    // pseudo_inverse(arm_jac.data, arm_jac_pinv);

    // // Getting required arm joint velocities
    // req_jvel = arm_jac_pinv * x_ref.segment(1, 6);

    // Integrating to get next position command
    // arm_traj.header.stamp = ros::Time::now() + ros::Duration(0, 150000000);
    // tmp_point.positions.clear();
    // arm_traj.points.clear();
    // tmp_point.positions.push_back(curr_js(0) + res);
    // tmp_point.positions.push_back(curr_js(1) + res);
    // tmp_point.positions.push_back(curr_js(2) + res);
    // tmp_point.positions.push_back(curr_js(3) + res);
    // tmp_point.positions.push_back(curr_js(4) + res);
    // tmp_point.positions.push_back(curr_js(5) + res);
    // tmp_point.positions.push_back(curr_js(6) + res);
    // tmp_point.time_from_start = ros::Duration(2, 0);
    // arm_traj.points.push_back(tmp_point);

    // hand_traj.header.stamp = ros::Time::now() + ros::Duration(0, 150000000);
    // tmp_point.positions.clear();
    // hand_traj.points.clear();
    // tmp_point.positions.push_back(curr_js(7) + res);
    // tmp_point.time_from_start = ros::Duration(2, 0);
    // hand_traj.points.push_back(tmp_point);

    // // Setting goal msgs
    // arm_goalmsg.trajectory = arm_traj;
    // hand_goalmsg.trajectory = hand_traj;

    // // Publishing to joint traj controllers
    // arm_client.sendGoal(arm_goalmsg);
    // hand_client.sendGoal(hand_goalmsg);

    // if(!arm_client.waitForResult(ros::Duration(10, 0))){
  	// 	ROS_WARN_STREAM("The arm is taking too much time to close! \n");
  	// }
    // if(!hand_client.waitForResult(ros::Duration(10, 0))){
  	// 	ROS_WARN_STREAM("The hand is taking too much time to close! \n");
  	// }

    // Print out all variables in contactPreserver
    // preserver.printAll();

    // Print out the resulting motion
    // std::cout << "Resulting reference motion x_ref is:" << std::endl;
    // std::cout << x_ref << std::endl;

    // Converting to geometry_msgs the twist of the palm and publishing
    cmd_twist.linear.x = x_ref(1); cmd_twist.angular.x = x_ref(4);
    cmd_twist.linear.y = x_ref(2); cmd_twist.angular.y = x_ref(5);
    cmd_twist.linear.z = x_ref(3); cmd_twist.angular.z = x_ref(6);

    cmd_syn.data = float (x_ref(0));

    // Getting computation time and couting
    duration_loop = ros::Time::now() - initial_time;
    ROS_DEBUG_STREAM("The computation time was " << duration_loop.toSec() << "s.");

    pub_cmd_hand.publish(cmd_syn);
    pub_cmd.publish(cmd_twist);

    // Sleeping for some time in order to execute the command before cancelling
    // ros::Duration(0.0001).sleep();

    // Publishing null commands for avoiding the repetition of old refs while computing
    // pub_cmd_hand.publish(cmd_syn_null);
    // pub_cmd.publish(cmd_twist_null);

    // Rate
    rate.sleep();
  }
}
