#include "robotCommander.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "robot_commander"
#define DEBUG             0   // print out additional info
#define DEBUG_PUB         1   // publishes additional info for rqt_plot

/**
* @brief The following are functions of the class robotCommander.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
robotCommander::robotCommander(std::string hand_topic_, std::string arm_topic_) : 
    ref_1_filter("double"), ref_2_filter("double"), ref_3_filter("double"), ref_4_filter("double"),
    ref_5_filter("double"), ref_6_filter("double"), ref_7_filter("double") {
    // Initializing service servers
    this->rc_server = this->nh_rc.advertiseService("rc_service", &robotCommander::performRobotCommand, this);
    this->emerg_server = this->nh_rc.advertiseService("rc_emergency_stop", &robotCommander::emergencyStop, this);

    // Setting emergency to false
    this->emergency = false;

    // Storing the topic strings
    this->hand_topic = hand_topic_; this->arm_topic = arm_topic_;
    
    // Initializing the publishers
    this->pub_hand = nh_rc.advertise<std_msgs::Float64>(this->hand_topic , 1);
    this->pub_arm = nh_rc.advertise<geometry_msgs::Twist>(this->arm_topic , 1);
    this->pub_twist_debug = nh_rc.advertise<geometry_msgs::WrenchStamped>("/rob_comm_twist_debug" , 1);
    this->pub_sigma_debug = nh_rc.advertise<std_msgs::Float64>("/rob_comm_sigma_debug" , 1);
    this->pub_twist_init = nh_rc.advertise<geometry_msgs::Twist>("/rob_comm_twist_init", 1);
    
    // Resizing the Eigen Vector
    this->x_ref.resize(7);
    this->filtered_x_ref.resize(7);

    // Setting up the filters
    this->ref_1_filter.configure("low_pass_filter", this->nh_rc);
    this->ref_2_filter.configure("low_pass_filter", this->nh_rc);
    this->ref_3_filter.configure("low_pass_filter", this->nh_rc);
    this->ref_4_filter.configure("low_pass_filter", this->nh_rc);
    this->ref_5_filter.configure("low_pass_filter", this->nh_rc);
    this->ref_6_filter.configure("low_pass_filter", this->nh_rc);
    this->ref_7_filter.configure("low_pass_filter", this->nh_rc);

    // Getting needed parameters
    if (!this->nh_rc.getParam("robot_commander/enable_filter", this->enable_filter)) {
        ROS_WARN("robotCommander::robotCommander : Could not get parameter enable_filter. Using default.");
    }
    if (!this->nh_rc.getParam("robot_commander/vel_limit", this->vel_limit)) {
        ROS_WARN("robotCommander::robotCommander : Could not get parameter vel_limit. Using default.");
    }
}

/* DESTRUCTOR */
robotCommander::~robotCommander(){
    // Nothing to do here
}

/* PERFORMROBOTCOMMAND */
bool robotCommander::performRobotCommand(adaptive_grasping::velCommand::Request &req,
    adaptive_grasping::velCommand::Response &res){
    // The bool to be returned and a tmp array var
    bool success = true;
    std::array<float, 7> x_ref_arr;

    // Saving the velocity reference in req to array and then to Eigen class variable
    std::copy(std::begin(req.x_ref), std::end(req.x_ref), std::begin(x_ref_arr));
    for(int i = 0; i < x_ref_arr.size(); i++){
        this->x_ref(i) = x_ref_arr[i];
    }

    if (DEBUG_PUB) {
        this->tmp_twist.linear.x = this->x_ref(0); this->tmp_twist.linear.y = this->x_ref(2); this->tmp_twist.linear.z = this->x_ref(3);
        this->tmp_twist.angular.x = this->x_ref(3); this->tmp_twist.angular.y = this->x_ref(4); this->tmp_twist.angular.z = this->x_ref(5);
        this->pub_twist_init.publish(this->tmp_twist);
    }
    
    // Debug message
    if(DEBUG) ROS_INFO_STREAM("robotCommander::performRobotCommand : The requested velocity vector is:" 
        << "\n" << this->x_ref << ".");

    // Checking if the reference contains NaNs
    if(this->x_ref.hasNaN()){
        ROS_FATAL_STREAM("robotCommander::performRobotCommand : The requested velocity vector contains NaNs, setting x_ref to null.");
        this->x_ref = Eigen::VectorXd::Zero(this->x_ref.size());
    }

    // Saving to hand_ref and palm_ref
    if(!emergency){
        this->setReferences(this->x_ref.head(1), this->x_ref.tail(6));
    } else {
        this->x_ref = Eigen::VectorXd::Zero(7);
    }

    // Checking velocity limits and eventually scaling them
    if(!this->enforceLimits(this->x_ref)){
        if(DEBUG){
            ROS_WARN("robotCommander::performRobotCommand : velocity limits violated, scaling the reference.");
        }
    }

    // Filtering the command in order to avoid peaks
    if(this->enable_filter){
        this->ref_1_filter.update(this->x_ref(0), this->filtered_x_ref(0));
        this->ref_2_filter.update(this->x_ref(1), this->filtered_x_ref(1));
        this->ref_3_filter.update(this->x_ref(2), this->filtered_x_ref(2));
        this->ref_4_filter.update(this->x_ref(3), this->filtered_x_ref(3));
        this->ref_5_filter.update(this->x_ref(4), this->filtered_x_ref(4));
        this->ref_6_filter.update(this->x_ref(5), this->filtered_x_ref(5));
        this->ref_7_filter.update(this->x_ref(6), this->filtered_x_ref(6));
    } else {
        this->filtered_x_ref = this->x_ref;
    }

    // Filling up the messages to be published
    this->cmd_syn.data = float (filtered_x_ref(0));
    this->cmd_twist.linear.x = filtered_x_ref(1); this->cmd_twist.angular.x = filtered_x_ref(4);
    this->cmd_twist.linear.y = filtered_x_ref(2); this->cmd_twist.angular.y = filtered_x_ref(5);
    this->cmd_twist.linear.z = filtered_x_ref(3); this->cmd_twist.angular.z = filtered_x_ref(6);

    // Publishing to robot controllers
    this->pub_hand.publish(this->cmd_syn);
    this->pub_arm.publish(this->cmd_twist);

    // Filling and publishing the twist for debug
    if(DEBUG_PUB) this->twist_wrench.header.frame_id = "world";
    if(DEBUG_PUB) this->twist_wrench.header.stamp = ros::Time::now();
    if(DEBUG_PUB) this->twist_wrench.wrench.force.x = filtered_x_ref(1); this->twist_wrench.wrench.torque.x = filtered_x_ref(4);
    if(DEBUG_PUB) this->twist_wrench.wrench.force.y = filtered_x_ref(2); this->twist_wrench.wrench.torque.y = filtered_x_ref(5);
    if(DEBUG_PUB) this->twist_wrench.wrench.force.z = filtered_x_ref(3); this->twist_wrench.wrench.torque.z = filtered_x_ref(6);
    if(DEBUG_PUB) this->pub_twist_debug.publish(this->twist_wrench);
    if(DEBUG_PUB) this->pub_sigma_debug.publish(this->cmd_syn);

    // Return the callback result
    res.success = success;
    return success;
}

/* ENFORCELIMITS */
bool robotCommander::enforceLimits(Eigen::VectorXd& vel_ref_){
    // Checking if any of the velocities in the vector is beyong limits
    bool violated = false; int index_viol; double biggest_viol = this->vel_limit;
    for(int i = 0; i < vel_ref_.size(); i++){
        if(std::abs(vel_ref_(i)) > biggest_viol){
            violated = true; index_viol = i; biggest_viol = std::abs(vel_ref_(i));
        }
    }

    // If limit has not been violated, return
    if(!violated){
        return true;
    }
    
    // Scale to drop the biggest violatio to the limit
    vel_ref_ = vel_ref_ * double(this->vel_limit / biggest_viol);
    return false;
    
}

/* SETREFERENCES */
void robotCommander::setReferences(Eigen::VectorXd hand_ref_,
    Eigen::VectorXd palm_ref_){
    // Setting the joint reference
    this->hand_ref = hand_ref_;

    // Converting palm_ref_ to geometry_msgs Twist and setting
    this->palm_ref.linear.x = palm_ref_(0); this->palm_ref.angular.x = palm_ref_(3);
    this->palm_ref.linear.y = palm_ref_(1); this->palm_ref.angular.y = palm_ref_(4);
    this->palm_ref.linear.z = palm_ref_(2); this->palm_ref.angular.z = palm_ref_(5);
}

/* EMERGENCYSTOP */
bool robotCommander::emergencyStop(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    // Setting the emergency bool
    if(req.data){
        ROS_ERROR_STREAM("robotCommander::emergencyStop EMERGENCY STOP REQUESTED! STOPPING ROBOT!");
    } else {
        ROS_WARN_STREAM("robotCommander::emergencyStop REACTIVATION REQUESTED! REMOVING STOP!");
    }
    this->emergency = req.data;

    // Setting the response
    res.success = req.data;
    if(req.data) res.message = "Successfully stopped.";
    else res.message = "Stop not requested. Emergency set to false. Robot enabled: be careful!!!";

    return true;
}
