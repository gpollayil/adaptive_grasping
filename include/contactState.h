#ifndef CONTACT_STATE_H
#define CONTACT_STATE_H

#include <map>
#include <tuple>
#include <Eigen/Dense>
#include <std_msgs/Int8.h>
#include <mutex>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>

// SERVICE INCLUDES
#include "finger_fk/FingerJointsService.h"

/**
* @brief This class is called by the adaptive_grasping method to get the
* state of the contacts (i.e. how many and which fingers are touching). This
* means that it listens to a topic for collisions, saves the ids and the values
* of the touching fingers and its frames (all inside the class)
*
*/

namespace adaptive_grasping {

  class contactState {

  public:

    /** DEFAULT CONSTRUCTOR
    * @brief Default constructor for contactState
    *
    * @param null
    * @return null
    */
    contactState();

    /** CONSTRUCTOR
    * @brief Overloaded constructor for contactState
    *
    * @param name of the topic where finger touch ids are published
    * @param a map containing correspondance id -> link_name
    * @param a map containing params that the main parsed from parameter server
    * @return null
    */
    contactState(std::string topic_name,
      std::map<int, std::string> link_names_map_,
        std::map<std::string, std::string> params_map_);

    /** DESTRUCTOR
    * @brief Default destructor for contactState
    *
    * @param null
    * @return null
    */
    ~contactState();

    // A boolean for checking if the object has been initialized
    bool initialized = false;

    /** INITIALIZE
    * @brief Private function to initialize the object
    *
    * @param name of the topic where finger touch ids are published
    * @param a map containing correspondance id -> link_name
    * @param a map containing params that the main parsed from parameter server
    * @return bool initialized status
    */
    bool intialize(std::string topic_name,
      std::map<int, std::string> link_names_map_,
        std::map<std::string, std::string> params_map_);

    /** READVALUES
    * @brief Class function to read the private variables
    *
    * @param maps
    * @return void
    */
    void readValues(std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>>& input_map_,
        std::map<int, sensor_msgs::JointState>& input_map2_);

    /** RESETCONTACTS
    * @brief Class function to reset the maps
    *
    * @param null
    * @return bool
    */
    bool resetContact();

  private:

    // A mutual exclusion lock for the variables of this class
    std::mutex contact_state_mutex;

    // The finger which has just touched (read via topic)
    int touching_finger;

    // Transform listener and stamped transform for lookupTransform
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_transform;

    // The map containing info on all the fingers in collision
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map;

    // The map containing the joint state of the touched fingers
    // Here we suppose that the finger_fk finger_joints_service is running
    std::map<int, sensor_msgs::JointState> joints_map;

    // Map for storing already read params from paramter server
    std::map<std::string, std::string> params_map;

    // Map for storing correspondance between id and finger link name
    std::map<int, std::string> link_names_map;

    // Node handle and subscriber for the subscriber to finger collision
    // and the service client for finger_joints_service
    ros::NodeHandle node_contact_state;
    ros::Subscriber finger_col_sub;
    ros::ServiceClient fj_client;

    /** HANDLECOLLISION
    * @brief Callback function to handle the touching topic (finger_col_sub)
    *
    * @param Int8 msg
    *   the listened message or the finger_col_sub
    * @return void (as all callbacks)
    */
    void handleCollision(const std_msgs::Int8::ConstPtr& msg);

    /** ITERATECONTACTS
    * @brief Auxiliary function to iterate and update the contacts_map
    *
    * @param Int8 null
    * @return void (does operation on class variables)
    */
    void iterateContacts();

    /** ITERATEJOINTS
    * @brief Auxiliary function to iterate and update the joints_map
    *
    * @param Int8 null
    * @return void (does operation on class variables)
    */
    void iterateJoints();

    /** GETTRANSFORM
    * @brief Class function to echo frame for the touching fingers
    *   which will be written an Eigen::Affine3d
    *
    * @param null
    * @return Eigen::Affine3d = transfrom of echoed frames
    */
    Eigen::Affine3d getTrasform(std::string frame1_name, std::string frame2_name);

  }; // closing class

} // closing namespace

#endif // CONTACT_STATE_H
