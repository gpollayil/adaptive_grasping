#ifndef FULL_GRASPER_H
#define FULL_GRASPER_H

#include "adaptiveGrasper.h"            // Most of other h files are included in this one

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

        /** INITIALIZE
        * @brief Public function for initializing the object (recalls the initialize function of adaptive_grasper)
        *
        * @param names_vector containing the names of all parameters needed
        * @return bool = true if success
        */
        bool initialize(std::vector<std::string> param_names);

        private:

        // The main element of the object
        adaptiveGrasper adaptive_grasper;

        // The velocity command to be sent to the robot commander (using the one of adaptive_grasper)
        velCommand handclose_command;

    };

}



#endif // FULL_GRASPER_H