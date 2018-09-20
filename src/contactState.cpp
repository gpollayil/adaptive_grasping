#include "contactState.h"

/**
* @brief The following are functions of the class contactState.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
contactState::contactState(std::string topic_name){
  // Subscribing to topic
  finger_col_sub = node_contact_state.subscribe(topic_name, 1000, &contactState::handleCollision, this);
}

/* HANDLECOLLISION */
void contactState::handleCollision(const std_msgs::Int8::ConstPtr& msg){
  // Trying to save msg  with and empty tuple (if already touching finger, then
  // won't insert)
  touching_finger = msg->data;
  Eigen::Affine3d
  std::tuple<std::string, Eigen::Affine3d,
    Eigen::Affine3d> empty (std::make_tuple(20,'b'))
}
