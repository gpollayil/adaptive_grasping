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
    // Subscribe to joint states
    this->js_sub = this->ag_nh.subscribe("joint_states", 1, &adaptiveGrasper::getJointsAndComputeSyn, this);
    ROS_WARN_STREAM("The subsciber subscribed to " << js_sub.getTopic() << ".");

    // Starting to parse the needed elements from parameter server
    this->initialized = this->ag_nh.getParam("adaptive_grasping", this->adaptive_params);
    if(this->initialized == false){
        ROS_ERROR_STREAM("adaptiveGrasper::initialize could not find the needed params");
    }
    this->initialized = this->parseParams(this->adaptive_params, param_names);
}

/* PARSEPARAMS */
bool adaptiveGrasper::parseParams(XmlRpc::XmlRpcValue params_xml, std::vector<std::string> param_names){
    // Starting to parse and save all needed single parameters
    parseParameter(params_xml, this->touch_topic_name, param_names[0]);
    parseParameter(params_xml, this->link_names_map, param_names[1]);
    parseParameter(params_xml, this->params_map, param_names[2]);
    parseParameter(params_xml, this->joint_numbers, param_names[3]);
    parseParameter(params_xml, this->H_i, param_names[4]);
    parseParameter(params_xml, this->S, param_names[5]);


}
