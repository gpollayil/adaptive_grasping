
#include <ros/ros.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

// MSG INCLUDES
#include <sensor_msgs/JointState.h>

// DEFINES
#define     DEBUG       0               // Prints out additional info

// GLOBAL VARIABLES
sensor_msgs::JointState::ConstPtr rb_joint_state;	// a msg where the subscriber will save the joint states


/**********************************************************************************************
 SUBSCRIBER CALLBACK
**********************************************************************************************/
void getJointStates(const sensor_msgs::JointState::ConstPtr &msg){
	// Storing the message into another global message variable
	rb_joint_state = msg;
}

/**********************************************************************************************
 MAIN
**********************************************************************************************/

int main(int argc, char** argv){
    // Init the ROS node
    ros::init(argc, argv, "collision_detection_twist_node");
    ros::NodeHandle nh_;

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
        bool use_octomap_monitor = false; // this prevents a /tf warning
        planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
        planning_scene_monitor_->startWorldGeometryMonitor("planning_scene", "world", false);
        planning_scene_monitor_->startStateMonitor("/" + group_name_ + "/joint_states");
        ROS_INFO_STREAM("Context monitors started for " << nh_.getNamespace());
    } else {
        ROS_ERROR_STREAM("Planning scene not configured for " << nh_.getNamespace());
    }

    // Creating collision request and result
    collision_detection::CollisionRequest creq;
    creq.group_name = group_name_;
    creq.distance = true;
    creq.verbose = true;
    collision_detection::CollisionResult cres;

    // Waiting for initial messages in joint_states
    ROS_INFO_STREAM("Waiting for first joint msg.");
    rb_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/" + group_name_ + "/joint_states");
    ROS_INFO_STREAM("Received first joint msg.");

    // Subscriber to joint states
    ros::Subscriber js_sub = nh_.subscribe("/" + group_name_ + "/joint_states", 1, getJointStates);

    ROS_INFO_STREAM("Created subscriber to joint states.");

    // Checking for collisions in ros loop
    while(ros::ok()){
        ros::spinOnce();

        // Getting the allowed collision matrix and print it
        collision_detection::AllowedCollisionMatrix acm = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
        std::ostream& stream = std::cout;

        // Getting all the entry names and couting
        std::vector<std::string> entry_names;
        acm.getAllEntryNames(entry_names);
        if(DEBUG){
            std::cout << "THE ACM ENTRY NAMES ARE:" << std::endl;
            for(std::vector<std::string>::const_iterator i = entry_names.begin(); i != entry_names.end(); ++i){
                std::cout << *i << " ";
            }
            std::cout << std::endl << "------------------" << std::endl;
        }

        // Setting the entries of acm corresponding to world to false
        acm.setEntry("world", false);
        if(DEBUG) acm.print(stream);  

        // Get the robot state
        robot_state::RobotState current_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();

        // Setting the robot state with current joint states
        for (int i = 0; i < rb_joint_state->position.size(); i++){
            // ROS_INFO_STREAM("Setting joint " << rb_joint_state->name[i] << " to " << rb_joint_state->position[i]);
            current_state.setJointPositions(rb_joint_state->name[i], &rb_joint_state->position[i]);
        }

        // Checking if the current planning scene has collision objects inside world
        current_state.printStatePositions(stream);
        planning_scene_monitor_->getPlanningScene()->printKnownObjects(stream);

        // Clearing the result and requesting for collision check
        cres.clear();
        planning_scene_monitor_->getPlanningScene()->checkCollision(creq, cres, current_state, acm);

        // Checking the result
        if(cres.collision){
            ROS_INFO_STREAM("State appears to be in collision with respect to group " << creq.group_name);

            // TODO: Stop Adaptive Grasping
        }
    }

    return 0;
}