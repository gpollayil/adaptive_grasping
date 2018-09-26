include "adaptiveGrasper.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "adaptive_grasper"

/**
* @brief The following are functions of the class adaptiveGrasper.
*
*/

using namespace adaptive_grasping;



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
