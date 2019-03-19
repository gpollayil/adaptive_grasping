#include "robotCommander.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "robot_commander"
#define DEBUG             0   // print out additional info

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
    this->pub_twist_debug = nh_rc.advertise<geometry_msgs::WrenchStamped>("/twist_debug" , 1);
    
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

}

/* DESTRUCTOR */
robotCommander::~robotCommander(){
    // Nothing to do here
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

    // Filtering the command in order to avoid peaks
    this->ref_1_filter.update(this->x_ref(0), this->filtered_x_ref(0));
    this->ref_2_filter.update(this->x_ref(1), this->filtered_x_ref(1));
    this->ref_3_filter.update(this->x_ref(2), this->filtered_x_ref(2));
    this->ref_4_filter.update(this->x_ref(3), this->filtered_x_ref(3));
    this->ref_5_filter.update(this->x_ref(4), this->filtered_x_ref(4));
    this->ref_6_filter.update(this->x_ref(5), this->filtered_x_ref(5));
    this->ref_7_filter.update(this->x_ref(6), this->filtered_x_ref(6));

    // Filling up the messages to be published
    this->cmd_syn.data = float (filtered_x_ref(0));
    this->cmd_twist.linear.x = filtered_x_ref(1); this->cmd_twist.angular.x = filtered_x_ref(4);
    this->cmd_twist.linear.y = filtered_x_ref(2); this->cmd_twist.angular.y = filtered_x_ref(5);
    this->cmd_twist.linear.z = filtered_x_ref(3); this->cmd_twist.angular.z = filtered_x_ref(6);

    // Publishing to robot controllers
    this->pub_hand.publish(this->cmd_syn);
    this->pub_arm.publish(this->cmd_twist);

    // Filling and publishing the twist for debug
    if(DEBUG) this->twist_wrench.header.frame_id = "world";
    if(DEBUG) this->twist_wrench.header.stamp = ros::Time::now();
    if(DEBUG) this->twist_wrench.wrench.force.x = filtered_x_ref(1); this->twist_wrench.wrench.torque.x = filtered_x_ref(4);
    if(DEBUG) this->twist_wrench.wrench.force.y = filtered_x_ref(2); this->twist_wrench.wrench.torque.y = filtered_x_ref(5);
    if(DEBUG) this->twist_wrench.wrench.force.z = filtered_x_ref(3); this->twist_wrench.wrench.torque.z = filtered_x_ref(6);
    if(DEBUG) this->pub_twist_debug.publish(this->twist_wrench);

    // Return the callback result
    res.success = success;
    return success;
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
