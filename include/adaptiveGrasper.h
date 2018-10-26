#ifndef ADAPTIVE_GRASPER_H
#define ADAPTIVE_GRASPER_H

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <utils/parsing_utilities.h>
#include "contactState.h"
#include "matricesCreator.h"
#include "contactPreserver.h"

/**
* @brief This class is created by the main of the adaptive_grasping_node: it
* contains all the other classes: contact_state, matrix_creator and contact_preserver
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

    /** OVERLOADED CONSTRUCTOR
    * @brief Overloaded constructor for adaptiveGrasper
    *
    * @param names_vector containing the names of all parameters needed
    * @return null
    */
    adaptiveGrasper(std::vector<std::string> param_names);

    /** DESTRUCTOR
    * @brief Default destructor for adaptiveGrasper
    *
    * @param null
    * @return null
    */
    ~adaptiveGrasper();

    // A boolean for checking if the object has been initialized
    bool initialized = false;

    /** INITIALIZE
    * @brief Public function for initializing the object
    *
    * @param names_vector containing the names of all parameters needed
    * @return bool = true if success
    */
    bool initialize(std::vector<std::string> param_names);

    /** PARSEPARAMS
    * @brief Class function to get a single param from parameter server
    *
    * @param names_vector containing the names of all parameters needed
    * @return bool = true if success
    */
    bool parseParams(XmlRpc::XmlRpcValue params_xml, std::vector<std::string> param_names);

  private:

    // ROS elements
    ros::NodeHandle ad_nh;

    // XMLRPC elements
    XmlRpc::XmlRpcValue adaptive_params;

    // A contactState element which manages the details about the contacts
    contactState my_contact_state;

    // A matrixCreator element which creates the needed matrices
    matricesCreator my_matrices_creator;

    // A contactPreserver element to compute contact preserving motions
    contactPreserver my_contact_preserver;


  };

}

#endif // ADAPTIVE_GRASPER_H
