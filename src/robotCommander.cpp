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
robotCommander::robotCommander(std::string hand_topic_, std::string arm_topic_){
  // Storing the topic strings
  this->hand_topic = hand_topic_; this->arm_topic = arm_topic_;

  // Initializing the action client and the publisher
  this->pub_hand = nh_rc.advertise<std_msgs::Float64>(this->hand_topic , 1);
  this->pub_arm = nh_rc.advertise<geometry_msgs::Twist>(this->arm_topic , 1);

  // Resizing the Eigen Vector
    this->x_ref.resize(7);
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
    ROS_INFO_STREAM("robotCommander::performRobotCommand : The requested velocity vector is:" 
        << "\n" << this->x_ref << ".");

    // Saving to hand_ref and palm_ref
    this->setReferences(this->x_ref.head(1), this->x_ref.tail(6));

    // Filling up the messages to be published
    this->cmd_syn.data = float (x_ref(0));
    this->cmd_twist.linear.x = x_ref(1); this->cmd_twist.angular.x = x_ref(4);
    this->cmd_twist.linear.y = x_ref(2); this->cmd_twist.angular.y = x_ref(5);
    this->cmd_twist.linear.z = x_ref(3); this->cmd_twist.angular.z = x_ref(6);

    // Publishing to robot controllers
    this->pub_hand.publish(this->cmd_syn);
    this->pub_arm.publish(this->cmd_twist);

    // Return the callback result
    res.success = success;
    return success;
}
