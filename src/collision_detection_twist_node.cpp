
#include <ros/ros.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

// MSG INCLUDES
#include <sensor_msgs/JointState.h>

// GLOBAL VARIABLES
sensor_msgs::JointState::ConstPtr rb_joint_state;	// a msg where the subscriber will save the joint states


/**********************************************************************************************
 SUBSCRIBER CALLBACK
**********************************************************************************************/
void getJointStates(const sensor_msgs::JointState::ConstPtr &msg){
	// Storing the message into another global message variable
	ROS_DEBUG_STREAM("GOT JOINTSTATE MSG: STARTING TO SAVE!");
	rb_joint_state = msg;
	ROS_DEBUG_STREAM("SAVED JOINTSTATE MSG!");
}

/**********************************************************************************************
 MAIN
**********************************************************************************************/

int main(int argc, char** argv){
    // Init the ROS node
    ros::init(argc, argv, "collision_detection_twist_node");
    ros::NodeHandle nh_("~");

    // Defining MoveIt Group name
    std::string group_name_;
    group_name_ = "right_arm";

    // Creating a robot model loader from robot_description
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

    // Getting the kinematic model and initializing a planning scene with it
    const robot_model::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // Creating a planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

    if(planning_scene_monitor_->getPlanningScene()){
        planning_scene_monitor_->startSceneMonitor("/planning_scene");
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();
        ROS_INFO_STREAM("Context monitors started for " << nh_.getNamespace());
    } else {
        ROS_ERROR_STREAM("Planning scene not configured for " << nh_.getNamespace());
    }
    
    // collision_detection::CollisionRequest collision_request;
    // collision_detection::CollisionResult collision_result;
    // planning_scene_monitor_->getPlanningScene()->checkSelfCollision(collision_request, collision_result);
    // ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Get the robot state
    robot_state::RobotState current_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();

    // Creating collision request and result
    collision_detection::CollisionRequest creq;
    creq.group_name = group_name_;
    creq.distance = true;
    creq.verbose = true;
    collision_detection::CollisionResult cres;

    // Waiting for initial messages in joint_states
    ROS_INFO_STREAM("Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>("/" + group_name_ + "/joint_states");
    ROS_INFO_STREAM("Received first joint msg.");

    // The subscriber for saving joint states
	ros::SubscribeOptions joint_state_so = ros::SubscribeOptions::create<sensor_msgs::JointState>("/" + group_name_ + "/joint_states", 
		1, getJointStates, ros::VoidPtr(), nh_.getCallbackQueue());
	ros::Subscriber js_sub = nh_.subscribe(joint_state_so);

    ros::spinOnce();

    // Checking for collisions in ros loop
    while(ros::ok()){
        // Setting the robot state with current joint states
        for (std::size_t i = 0; i < rb_joint_state->position.size(); ++i){
            current_state.setJointPositions(rb_joint_state->name[i], &rb_joint_state->position[i]);
        }

        // Clearing the result and requesting for collision check
        cres.clear();
        planning_scene_monitor_->getPlanningScene()->checkCollision(creq, cres, current_state);

        // Checking the result
        if(cres.collision){
            ROS_INFO_STREAM("State appears to be in collision with respect to group " << creq.group_name);

            // TODO: Stop Adaptive Grasping
        }
    }

    return 0;
}