#include "contactState.h"

/**
* @brief The following are functions of the class contactState.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
contactState::contactState(){
  // Getting params from parameter server
  std::string topic_name = "finger_collision_topic";
  std::string default_topic_name = "/touching_finger_topic";
  if(!getParamOfYaml(topic_name, default_topic_name)){
    ROS_WARN_STREAM(topic_name << " not found! Please load the correct one from yaml!");
  }

  // Subscribing to topic
  finger_col_sub = node_contact_state.subscribe(topic_name, 1000, &contactState::handleCollision, this);
}

/* HANDLECOLLISION */
void contactState::handleCollision(const std_msgs::Int8::ConstPtr& msg){

}

/* GETPARAMSOFYAML */
bool contactState::getParamOfYaml(std::string param_name, std::string default_param_name){
  bool success = true;

  // Looking if parameter exists and loading it
  if(!ros::param::get(std::string(EXEC_NAMESPACE) +
      std::string(CLASS_NAMESPACE) + param_name, this->temp_param)){
        ROS_WARN_STREAM(param_name << " param not found in param server! Using default.");

        // Saving default
        this->temp_param = default_param_name;

        success = false;
	}

  // Inserting in map
  params_map[param_name] = this->temp_param;

  return success;
}
