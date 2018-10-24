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
robotCommander::robotCommander(std::string hand_topic_, std::string arm_topic_,
  std::vector<std::string> joint_names_vec_){
  // Storing the topic strings
  this->hand_topic = hand_topic_; this->arm_topic = arm_topic_;

  // Storing the joint names vector
  this->joint_names_vec = joint_names_vec_;

  // Initializing the action client and the publisher
  this->pub_hand = nh_rc.advertise<std_msgs::Float64>(this->hand_topic , 1);
  this->pub_arm = nh_rc.advertise<geometry_msgs::Twist>(this->arm_topic , 1);

  // Initializing subscriber to joint states
  this->joint_state_sub = nh_rc.subscribe("joint_states", 1,
    &robotCommander::getJointStates, this);
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

  // Setting only starting previous time
  this->prev_time = ros::Time::now();
}
