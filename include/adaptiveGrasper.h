#ifndef ADAPTIVE_GRASPER_H
#define ADAPTIVE_GRASPER_H

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "utils/parsing_utilities.h"
#include "contactState.h"
#include "matricesCreator.h"
#include "contactPreserver.h"

// Service Includes
#include "adaptive_grasping/velCommand.h"

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

    /** PRINTPARSED
    * @brief Public function for printing the parsed data
    *
    * @param null
    * @return null
    */
    void printParsed();

    /** PRINTCONTACTSINFO
    * @brief Public function for printing the contacts data
    *
    * @param null
    * @return null
    */
    void printContactsInfo();

    /** PRINTOBJECTPOSE
    * @brief Public function for printing the object pose
    *
    * @param null
    * @return null
    */
    void printObjectPose();

    /** SPINGRASPER
    * @brief Public function to run the adaptive loop
    *
    * @param null
    * @return null
    */
    void spinGrasper();

  private:

    // ROS elements
    ros::NodeHandle ag_nh;
    ros::Subscriber js_sub;                               // Subscriber to joint states
    ros::Subscriber op_sub;                               // Subscriber to object pose
    ros::ServiceClient client_rc;                         // Service client to robot commander
    double spin_rate;                                     // Rate at which the adaptive grasper should run

    // Message variables
    sensor_msgs::JointState::ConstPtr full_joint_state;	  // A msg where the subscriber will save the joint states

    // XMLRPC elements
    XmlRpc::XmlRpcValue adaptive_params;

    // A mutual exclusion lock for the variables of this class
    std::mutex adaptive_grasper_mutex;

    // Elements needed for construction of the main objects
    std::string touch_topic_name;                       // Contains the name of the topic where touch ids are published (for Contact State)
    std::map<int, std::string> link_names_map;          // Contains the correspondance between ids and link names (for Contact State)
    std::map<std::string, std::string> params_map;      // Contains mainly frame names (for Contact State)
    std::vector<int> joint_numbers;                     // Contains the number of links of each finger (for Matrix Creator)
    Eigen::MatrixXd H_i;                                // Contains contact selection matrix H (for Matrix Creator)
    Eigen::MatrixXd S;                                  // Contains the synergy matrix (for Contact Preserver)
    Eigen::MatrixXd A_tilde;                            // Contains the weigth matrix (for Contact Preserver)
    Eigen::VectorXd x_d;                                // Contains the desired x motion (for Contact Preserver)
    Eigen::VectorXd x_ref;                              // Contains the result of minimization x reference (from Contact Preserver)
    std::string object_topic_name;                      // Contains the name of the topic where the object poses are published (for Matrix Creator)
    double scaling;                                     // Contains the scaling factor for x reference

    // A contactState element which manages the details about the contacts
    contactState my_contact_state;

    // A matrixCreator element which creates the needed matrices
    matricesCreator my_matrices_creator;

    // A contactPreserver element to compute contact preserving motions
    contactPreserver my_contact_preserver;

    // The maps written by contact state
    std::map<int, std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>> read_contacts_map;
    std::map<int, sensor_msgs::JointState> read_joints_map;

    // The object pose which is updated by a subscriber to a topic
    Eigen::Affine3d object_pose;

    // The main matrices created by matrices creator and used for minimization
    Eigen::MatrixXd read_J; Eigen::MatrixXd read_G;
    Eigen::MatrixXd read_T; Eigen::MatrixXd read_H;

    // The service file to be sent to the robot commander server
    adaptive_grasping::velCommand ref_command;

    /** PARSEPARAMS
    * @brief Class function to get a single param from parameter server
    *
    * @param names_vector containing the names of all parameters needed
    * @return bool = true if success
    */
    bool parseParams(XmlRpc::XmlRpcValue params_xml, std::vector<std::string> param_names);

    /** SETCOMMANDANDSEND
    * @brief Class function to clear the service file, push back the new ref and send to robot commander
    *
    * @param ref_vec eigen::vector of reference motions
    * @param comm the service file to be filled and sent
    * @return bool = true if success
    */
    bool setCommandAndSend(Eigen::VectorXd ref_vec, adaptive_grasping::velCommand comm);

    /** GETJOINTSANDCOMPUTESYN
    * @brief Callback function to get the joint states and compute the synergy matrix
    *
    * @param msg
    * @return null
    */
    void getJointsAndComputeSyn(const sensor_msgs::JointState::ConstPtr &msg);

    /** GETOBJECTPOSE
    * @brief Callback function to get the object pose from a topic
    *
    * @param msg
    * @return null
    */
    void getObjectPose(const geometry_msgs::Pose::ConstPtr &msg);

  };

}

#endif // ADAPTIVE_GRASPER_H
