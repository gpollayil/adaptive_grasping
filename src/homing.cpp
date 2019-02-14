// ROS INCLUDES
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <controller_manager_msgs/SwitchController.h>
#include "adaptive_grasping/adaptiveGrasp.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>


#define DEBUG			0										// if DEBUG 1 prints additional couts

// GLOBAL VARIABLES

// AUX FUNCTION
bool homing_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    if(!req.data){
        res.message = "No homing requested";
        res.success = false;
        return false;
    }

    adaptive_grasping::adaptiveGrasp adaptive_srv;
    adaptive_srv.request.run_adaptive_grasp = false;

    if(!ros::service::call<adaptive_grasping::adaptiveGrasp>("adaptive_grasper_service", adaptive_srv)){
        ROS_ERROR_STREAM("Could not stop adaptive grasping!");
    }

    std_srvs::SetBool stop_srv;
    stop_srv.request.data = true;

    if(!ros::service::call<std_srvs::SetBool>("rc_emergency_stop", stop_srv)){
        ROS_ERROR_STREAM("Could not stop robot commander!");
    }

    controller_manager_msgs::SwitchController switch_controller;

    // Clearing the switch message
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();

    // Filling up the switch message
    switch_controller.request.start_controllers.push_back("joint_trajectory_controller");
    switch_controller.request.stop_controllers.push_back("twist_controller");
    switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    // Swithching controller by calling the service
    if(!ros::service::call<controller_manager_msgs::SwitchController>("right_arm/controller_manager/switch_controller", switch_controller)){
        ROS_ERROR_STREAM("Could not switch arm controllers!");
    }

    // Clearing the switch message
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();

    // Filling up the switch message
    switch_controller.request.start_controllers.push_back("joint_trajectory_controller");
    switch_controller.request.stop_controllers.push_back("velocity_controller");
    switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    // Swithching controller by calling the service
    if(!ros::service::call<controller_manager_msgs::SwitchController>("right_hand/controller_manager/switch_controller", switch_controller)){
        ROS_ERROR_STREAM("Could not switch hand controllers!");
    }

    // Joint traj
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client("/right_arm/joint_trajectory_controller/follow_joint_trajectory/", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> hand_client("/right_hand/joint_trajectory_controller/follow_joint_trajectory/", true);

	// Create the goal for hand
	control_msgs::FollowJointTrajectoryGoal hand_goalmsg;
	trajectory_msgs::JointTrajectory trajectory_hand;
    ros::Duration current_duration(double (0.0));

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(double(0.25));

    point.time_from_start = current_duration;
    trajectory_hand.points.push_back(point);
    trajectory_hand.joint_names.push_back("right_hand_synergy_joint");

    hand_goalmsg.trajectory = trajectory_hand;

    // Create the goal for hand
    control_msgs::FollowJointTrajectoryGoal arm_goalmsg;
	trajectory_msgs::JointTrajectory trajectory_arm;

    trajectory_msgs::JointTrajectoryPoint arm_point;
    point.positions.push_back(double(0.25));

    trajectory_arm.points.push_back(arm)

  // Create the joint names
  ROS_INFO_STREAM("Handlling joints:");
  std::string buff;

  // Push back of lwr joint names
  buff = "right_arm_a1_joint";
  ROS_INFO_STREAM(buff);
  trajectory.joint_names.push_back(buff);
  buff = "right_arm_a2_joint";
  ROS_INFO_STREAM(buff);
  trajectory.joint_names.push_back(buff);
  buff = "right_arm_e1_joint";
  ROS_INFO_STREAM(buff);
  trajectory.joint_names.push_back(buff);
  buff = "right_arm_a3_joint";
  ROS_INFO_STREAM(buff);
  trajectory.joint_names.push_back(buff);
  buff = "right_arm_a4_joint";
  ROS_INFO_STREAM(buff);
  trajectory.joint_names.push_back(buff);
  buff = "right_arm_a5_joint";
  ROS_INFO_STREAM(buff);
  trajectory.joint_names.push_back(buff);
  buff = "right_arm_a6_joint";
  ROS_INFO_STREAM(buff);
  trajectory.joint_names.push_back(buff);

	// Set the created trajectory
	goalmsg.trajectory = trajectory;
	client.sendGoal(goalmsg);

}

// MAIN
int main(int argc, char** argv){
    
    // Initializing ROS node
	ros::init(argc, argv, "homing_node");
	ros::NodeHandle home_nh;

    // Server for this node
    ros::ServiceServer homing_server = home_nh.advertiseService("homing_service", homing_callback);

	// Success message
	std::cout << "TF Listener started to work! The TF info are being published" << std::endl;

	// Spin
	ros::spin ();

}
