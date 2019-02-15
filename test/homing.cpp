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
const double right_hand_synergy_joint_value = 0.30;
const double right_arm_a1_joint_value = 0.40 ;
const double right_arm_a2_joint_value = -1.09;
const double right_arm_a3_joint_value = -1.59;
const double right_arm_a4_joint_value = -0.60;
const double right_arm_a5_joint_value = -0.25;
const double right_arm_a6_joint_value = 0.40;
const double right_arm_e1_joint_value = -0.29;

// AUX FUNCTION
bool homing_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    if(!req.data){
        res.message = "No homing requested";
        res.success = false;
        return false;
    }

    // Stopping adaptive grasp (this->run in adaptiveGrasper.cpp)
    adaptive_grasping::adaptiveGrasp adaptive_srv;
    adaptive_srv.request.run_adaptive_grasp = false;
    if(!ros::service::call<adaptive_grasping::adaptiveGrasp>("adaptive_grasper_service", adaptive_srv)){
        ROS_ERROR_STREAM("Could not stop adaptive grasping!");
        res.message = "Could not stop adaptive grasping!";
        res.success = false;
        return false;
    }

    // Stopping robot commander with emergency stop
    std_srvs::SetBool stop_srv;
    stop_srv.request.data = true;
    if(!ros::service::call<std_srvs::SetBool>("rc_emergency_stop", stop_srv)){
        ROS_ERROR_STREAM("Could not stop robot commander!");
        res.message = "Could not stop robot commander!";
        res.success = false;
        return false;
    }

    // Starting to switch controller from twist controller to joint trajectory controller for arm
    controller_manager_msgs::SwitchController switch_controller; 
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();
    switch_controller.request.start_controllers.push_back("joint_trajectory_controller");
    switch_controller.request.stop_controllers.push_back("twist_controller");
    switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    if(!ros::service::call<controller_manager_msgs::SwitchController>("right_arm/controller_manager/switch_controller", switch_controller)){
        ROS_ERROR_STREAM("Could not switch arm controllers!");
        res.message = "Could not switch arm controllers!";
        res.success = false;
        return false;
    }

    // Starting to switch controller from velocity controller to joint trajectory controller for hand
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();
    switch_controller.request.start_controllers.push_back("joint_trajectory_controller");
    switch_controller.request.stop_controllers.push_back("velocity_controller");
    switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    if(!ros::service::call<controller_manager_msgs::SwitchController>("right_hand/controller_manager/switch_controller", switch_controller)){
        ROS_ERROR_STREAM("Could not switch hand controllers!");
        res.message = "Could not switch hand controllers!";
        res.success = false;
        return false;
    }

    // Joint trajectory action clients
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client("/right_arm/joint_trajectory_controller/follow_joint_trajectory/", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> hand_client("/right_hand/joint_trajectory_controller/follow_joint_trajectory/", true);

	// Create the goal for hand and for the arm
	control_msgs::FollowJointTrajectoryGoal hand_goalmsg;
	trajectory_msgs::JointTrajectory trajectory_hand;
    trajectory_hand.header.stamp = ros::Time::now() + ros::Duration(1, 0);

    control_msgs::FollowJointTrajectoryGoal arm_goalmsg;
	trajectory_msgs::JointTrajectory trajectory_arm;
    trajectory_arm.header.stamp = ros::Time::now() + ros::Duration(1, 0);
    
    // Assign the headers
    trajectory_hand.header.stamp = ros::Time::now() + ros::Duration(1.0);
    trajectory_arm.header.stamp = ros::Time::now() + ros::Duration(1.0);

    // Create a generic point which will be used several times
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(double(1.0));

    // Give waypoint for hand
    trajectory_hand.joint_names.push_back("right_hand_synergy_joint");
    point.positions.clear();
    point.positions.push_back(double(right_hand_synergy_joint_value));
    trajectory_hand.points.push_back(point);

    hand_goalmsg.trajectory = trajectory_hand;

    // Push back of lwr joint names and positions
    trajectory_arm.joint_names.clear();
    point.positions.clear();

    trajectory_arm.joint_names.push_back("right_arm_a1_joint");
    trajectory_arm.joint_names.push_back("right_arm_a2_joint");
    trajectory_arm.joint_names.push_back("right_arm_e1_joint");
    trajectory_arm.joint_names.push_back("right_arm_a3_joint");
    trajectory_arm.joint_names.push_back("right_arm_a4_joint");
    trajectory_arm.joint_names.push_back("right_arm_a5_joint");
    trajectory_arm.joint_names.push_back("right_arm_a6_joint");
    

    point.positions.push_back(right_arm_a1_joint_value);
    point.positions.push_back(right_arm_a2_joint_value);
    point.positions.push_back(right_arm_e1_joint_value);
    point.positions.push_back(right_arm_a3_joint_value);
    point.positions.push_back(right_arm_a4_joint_value);
    point.positions.push_back(right_arm_a5_joint_value);
    point.positions.push_back(right_arm_a6_joint_value);
    
    trajectory_arm.points.push_back(point);

	arm_goalmsg.trajectory = trajectory_arm;

    // Sending the trajectory to clients
    ROS_INFO_STREAM("Going back to home!");
    if (arm_client.waitForServer(ros::Duration(1,0))){
        arm_client.sendGoal(arm_goalmsg);
    } else {
      	ROS_WARN_STREAM("No arm server available. Are you running in simulation without Gazebo?");
    }
    if (hand_client.waitForServer(ros::Duration(1,0))){
        hand_client.sendGoal(hand_goalmsg);
    } else {
      	ROS_WARN_STREAM("No hand server available. Are you running in simulation without Gazebo?");
    }

    ROS_INFO_STREAM("Sleeping for result!");
    if(!arm_client.waitForResult(ros::Duration(double(10.0)))) ROS_ERROR_STREAM("Could not reach the goal for arm!");
    if(!hand_client.waitForResult(ros::Duration(double(10.0)))) ROS_ERROR_STREAM("Could not reach the goal for hand!");;

    // Swithching back the controllers
    ROS_INFO_STREAM("Switching back to velocity controllers and re-enabling robot commander!");

    // Starting to switch controller from joint trajectory controller to twist controller for arm
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();
    switch_controller.request.start_controllers.push_back("twist_controller");
    switch_controller.request.stop_controllers.push_back("joint_trajectory_controller");
    switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    if(!ros::service::call<controller_manager_msgs::SwitchController>("right_arm/controller_manager/switch_controller", switch_controller)){
        ROS_ERROR_STREAM("Could not switch arm controllers!");
        res.message = "Could not switch back the hand controllers! But not a big failure...";
    }

    // Starting to switch controller from joint trajectory controller to velocity controller for hand
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();
    switch_controller.request.start_controllers.push_back("velocity_controller");
    switch_controller.request.stop_controllers.push_back("joint_trajectory_controller");
    switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    if(!ros::service::call<controller_manager_msgs::SwitchController>("right_hand/controller_manager/switch_controller", switch_controller)){
        ROS_ERROR_STREAM("Could not switch hand controllers!");
        res.message = "Could not switch back the hand controllers! But not a big failure...";
    }

    // Removing the stop in robot commander
    std_srvs::SetBool cancel_stop_srv;
    cancel_stop_srv.request.data = false;
    if(!ros::service::call<std_srvs::SetBool>("rc_emergency_stop", cancel_stop_srv)){
        ROS_ERROR_STREAM("Could not restart robot commander!");
        res.message = "Could not restart robot commander!";
    }

    ROS_WARN_STREAM("Finished Homing! PLEASE RESTART THE ADAPTIVE GRASPING FOR RESETTING THE CONTACTS!");

    res.success = true;
    return true;
}

// MAIN
int main(int argc, char** argv){
    
    // Initializing ROS node
	ros::init(argc, argv, "homing_node");
	ros::NodeHandle home_nh;

    // Server for this node
    ros::ServiceServer homing_server = home_nh.advertiseService("homing_service", homing_callback);

	// Success message
	std::cout << "Homing node started to work! Waiting for the service to be called." << std::endl;

	// Spin
	ros::spin ();

}
