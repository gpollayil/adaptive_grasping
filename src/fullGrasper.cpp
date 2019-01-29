#include "fullGrasper.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "full_grasper"

#define DEBUG   0           // Prints out additional info (additional to ROS_DEBUG)

/**
* @brief The following are functions of the class fullGrasper.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
fullGrasper::fullGrasper(){
    // Nothing to do here
}

/* INITIALIZE */
bool fullGrasper::initialize(std::vector<std::string> param_names){
    // Using the initialize of adaptiveGrasper
    return this->adaptive_grasper.initialize(param_names);
}

