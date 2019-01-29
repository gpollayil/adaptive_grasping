#ifndef FULL_GRASPER_H
#define FULL_GRASPER_H

#include "adaptiveGrasper.h"            // Most of other h files are included in this one

#include <controller_manager_msgs/SwitchController.h>

/**
* @brief This class is created by the main of the full_grasping_node: it has inside it
* all the other classes: contact_state, matrix_creator, contact_preserver and adaptive_grasper.
* The purpouse of this node is to start closing the hand until a touch is detected; then the
* callback of the adaptive_grasper is called
*
*/

namespace adaptive_grasping {

    class fullGrasper {
        
        public:

        /** CONSTRUCTOR
        * @brief Default constructor for fullGrasper
        *
        * @param null
        * @return null
        */
        fullGrasper();

        /** OVERLOADED CONSTRUCTOR
        * @brief Default constructor for fullGrasper
        *
        * @param robot_ns_ a vector containing the strings of the namespaces of robots (kuka and hand?)
        * @param normal_controllers_names_ a vector containing the strings of the normal controllers
        * @param velocity_controllers_names_ a vector containing the strings of the velocity controllers
        * @return null
        */
        fullGrasper(std::vector<std::string> robot_ns_, std::vector<std::string> normal_controllers_names_, std::vector<std::string> velocity_controllers_names_);

        /** INITIALIZE
        * @brief Public function for initializing the object (recalls the initialize function of adaptive_grasper)
        *
        * @param names_vector containing the names of all parameters needed
        * @return bool = true if success
        */
        bool initialize(std::vector<std::string> param_names);

        /** SWITCHCONTROL
        * @brief Public function for switching from a controller to another
        *
        * @param robot_name containing the name of robot
        * @param from_controller containing the name of controller to be stopped
        * @param to_controller containing the name of controller to be started
        * @return bool = true if success
        */
        bool switch_control(std::string robot_name, std::string from_controller, std::string to_controller);

        private:

        // The main element of the object
        adaptiveGrasper adaptive_grasper;

        // The velocity command to be sent to the robot commander (using the one of adaptive_grasper)
        velCommand handclose_command;

        // The namespace of the robot controllers (in order kuka and soft_hand)
        std::vector<std::string> robot_namespaces;

        // The switch controller service name
        std::string switch_service_name = "/controller_manager/switch_controller";

        // The names of the normal controllers and the velocity controllers (in order kuka and soft_hand)
        std::vector<std::string> normal_controllers_names;
        std::vector<std::string> velocity_controllers_names;

        // A controller_mangager msg for switching controllers
        controller_manager_msgs::SwitchController switch_controller;

    };

}



#endif // FULL_GRASPER_H