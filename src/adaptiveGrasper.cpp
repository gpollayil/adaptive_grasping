#include "adaptiveGrasper.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "adaptive_grasper"

/**
* @brief The following are functions of the class adaptiveGrasper.
*
*/

using namespace adaptive_grasping;

/* OVERLOADED CONSTRUCTOR */
adaptiveGrasper::adaptiveGrasper(std::vector<std::string> param_names){
    // Building the object
    this->initialized = this->initialize(param_names);
}

/* DESTRUCTOR */
adaptiveGrasper::~adaptiveGrasper(){
    // Nothing to do here now
}

/* INITIALIZE */
bool adaptiveGrasper::initialize(std::vector<std::string> param_names){
    // Starting to parse the needed elements from parameter server
    this->initialized = this->ad_nh.getParam("adaptive_grasping", this->adaptive_params);
    if(this->initialized == false){
        ROS_ERROR_STREAM("adaptiveGrasper::initialize could not find the needed params");
    }
    this->initialized = this->parseParams(this->adaptive_params, param_names);
}

/* PARSEPARAMS */
bool adaptiveGrasper::parseParams(XmlRpc::XmlRpcValue params_xml, std::vector<std::string> param_names){
    
}
