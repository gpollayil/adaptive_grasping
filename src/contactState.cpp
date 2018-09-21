#include "contactState.h"

/**
* @brief The following are functions of the class contactState.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
contactState::contactState(std::string topic_name,
  std::map<int, std::string> link_names_map_,
    std::map<std::string, std::string> params_map_){
  // Subscribing to topic
  finger_col_sub = node_contact_state.subscribe(topic_name, 1000,
    &contactState::handleCollision, this);

  // Constructing the maps
  this->params_map = params_map_;
  this->link_names_map = link_names_map_;
}

/* HANDLECOLLISION */
void contactState::handleCollision(const std_msgs::Int8::ConstPtr& msg){
  // Temporary saving of message
  touching_finger = msg->data;

  // Creating an tuple with identity Affine3ds
  Eigen::Affine3d identity_aff = Eigen::Affine3d::Identity();
  std::string touched_link_name = link_names_map.at(touching_finger);
  std::tuple<std::string, Eigen::Affine3d,
    Eigen::Affine3d> empty_tuple (std::make_tuple(touched_link_name,
      identity_aff, identity_aff));

// Just to be sure that the last touched finger id is in the map, inserting
// (eventually overwriting) the element in the contacts_map
contact_state_mutex.lock();                             // mutex on
contacts_map[touching_finger] = empty_tuple;
contact_state_mutex.unlock();                           // mutex off

// Creating an iterator for contacts_map
std::map<int, std::tuple<std::string, Eigen::Affine3d,
  Eigen::Affine3d>>::iterator it_contacts = contacts_map.begin();

// Now with a loop echoing and saving all needed transforms in contacts_map

}
