#include "robotCommander.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "robot_commander"
#define DEBUG             1   // print out additional info

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
  act_hand = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>
    (this->hand_topic, true);
  pub_arm = nh_rc.advertise<geometry_msgs::Twist>(this->arm_topic , 1);
}

/* DESTRUCTOR */
robotCommander::~robotCommander(){
  // Nothing to do here
}
