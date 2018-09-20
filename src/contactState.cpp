#include "contactState.h"

/**
* @brief The following are functions of the class contactState.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
contactState::contactState(){
  finger_col_sub = node_contact_state.subscribe("/" + std::string(IMU_TOPIC), 1000, fingerColCallback);s
}
