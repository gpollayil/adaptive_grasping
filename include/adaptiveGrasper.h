#ifndef ADAPTIVE_GRASPER_H
#define ADAPTIVE_GRASPER_H

#include "contactState.h"
#include "matricesCreator.h"
#include "contactPreserver.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "adaptive_grasper"

/**
* @brief This class is created by the main of the adaptive_grasping_node: it
* contains all the other classes: contact_state, matrix_creator,
* contact_preserver and robot_commander
*
*/

namespace adaptive_grasping {

  class adaptiveGrasper {

  public:

    /** CONSTRUCTOR
    * @brief Default constructor for adaptiveGrasper
    *
    * @param null
    * @return null
    */
    adaptiveGrasper();

    /** DESTRUCTOR
    * @brief Default destructor for adaptiveGrasper
    *
    * @param null
    * @return null
    */
    ~adaptiveGrasper();

    /** GETPARAMSOFYAML
    * @brief Class function to get a single param from parameter server
    *
    * @param  param_name: name of the parameter to be parsed
    * @param  default_param_name: default to be saved if param not found
    * @return bool = true if success
    */
    bool getParamOfYaml(std::string param_name, std::string default_param_name);

    /** PARSEPARAMS
    * @brief Class function to get a single param from parameter server
    *
    * @param  nh: node handle
    * @return bool = true if success
    */
    bool parseParams(ros::NodeHandle nh);

  private:

    // A contactState element which manages the details about the contacts
    contactState my_contact_state;

    // A matrixCreator element which creates the needed matrices
    matricesCreator my_matrices_creator;

    // A contactPreserver element to compute contact preserving motions
    contactPreserver my_contact_preserver;

    // A robotCommander element for passing references to robot


  };

}

#endif // ADAPTIVE_GRASPER_H
