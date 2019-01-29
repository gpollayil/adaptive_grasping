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

/* OVERLOADED CONSTRUCTOR */
fullGrasper::fullGrasper(std::vector<std::string> robot_ns_, std::vector<std::string> normal_controllers_names_, std::vector<std::string> velocity_controllers_names_){
    // Setting the robot controllers namespace
    this->robot_namespaces = robot_ns_;

    // Setting the names of the controllers
    this->normal_controllers_names = normal_controllers_names_;
    this->velocity_controllers_names = velocity_controllers_names_;
}

/* INITIALIZE */
bool fullGrasper::initialize(std::vector<std::string> param_names){
    // Using the initialize of adaptiveGrasper
    return this->adaptive_grasper.initialize(param_names);
}

/* SWITCHCONTROL */
bool fullGrasper::switch_control(std::string robot_name, std::string from_controller, std::string to_controller){
    // Temporary bool to be returned
    bool success = false;

    // Clearing the switch message
    this->switch_controller.request.start_controllers.clear();
    this->switch_controller.request.stop_controllers.clear();

    // Filling up the switch message
    this->switch_controller.request.start_controllers.push_back(to_controller);
    this->switch_controller.request.stop_controllers.push_back(from_controller);
    this->switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    // Swithching controller by calling the service
    ros::service::call<controller_manager_msgs::SwitchController>(robot_name + this->switch_service_name, this->switch_controller);


}


