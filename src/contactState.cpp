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
  Eigen::Affine3d>>::iterator it_c;

// Now with a loop echoing and saving all needed transforms in contacts_map
for(it_c = contacts_map.begin(); it_c != contacts_map.end(); ++it_c){
  // Getting the frame names
  std::string frame_fing = std::get<0>(it_c->second);
  std::string frame_world = params_map.at("world_name");
  std::string frame_palm = params_map.at("palm_name");

  // Getting all the needed transforms
  Eigen::Affine3d fing_aff = getTrasform(frame_world, frame_fing);
  Eigen::Affine3d palm_aff = getTrasform(frame_fing, frame_palm);

  // Writing the correct tuple into the map
  std::tuple<std::string, Eigen::Affine3d,
    Eigen::Affine3d> correct_tuple (std::make_tuple(frame_fing,
      fing_aff, palm_aff));
  contact_state_mutex.lock();                             // mutex on
  contacts_map[touching_finger] = correct_tuple;
  contact_state_mutex.unlock();                           // mutex off

}
}

/* GETTRANSFORM*/
Eigen::Affine3d contactState::getTrasform(std::string frame1_name,
  std::string frame2_name){

}
