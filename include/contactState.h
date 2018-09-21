#ifndef CONTACT_STATE_H
#define CONTACT_STATE_H

#include <map>
#include <tuple>
#include <Eigen/Geometry>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <mutex>

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "contact_state"

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

    // public typedefs ???
    //typedef boost::shared_ptr<contactState> Ptr;
    //typedef boost::shared_ptr<const contactState> ConstPtr;

    /** CONSTRUCTOR
    * @brief Default constructor for contactState
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

    /** READVALUES
    * @brief Class function to read the private variables
    *
    * @param null
    * @return std::map< int, Eigen::Affine3d >
    */
    bool readValues(std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>>& contacts_map);

  private:

    // A mutual exclusion lock for the variables of this class
    std::mutex contact_state_mutex;

    // The finger which has just touched (read via topic)
    int touching_finger;

    // Temporary string for saving parameters (Needed???)
    std::string temp_param;

    // The vector containing info on all the fingers in collision
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map;

    // Map for storing already read params from paramter server
    std::map<std::string, std::string> params_map;

    // Map for storing correspondance between id and finger link name
    std::map<int, std::string> link_names_map;

    // Node handle and subscriber for the subscriber to finger collision
    ros::NodeHandle node_contact_state;
    ros::Subscriber finger_col_sub;

    /** HANDLECOLLISION
    * @brief Callback function to handle the touching topic (finger_col_sub)
    *
    * @param Int8 msg
    *   the listened message or the finger_col_sub
    * @return void (as all callbacks)
    */
    void handleCollision(const std_msgs::Int8::ConstPtr& msg);

    /** GETTRANSFORM
    * @brief Class function to echo frame for the touching fingers
    *   which will be written an Eigen::Affine3d
    *
    * @param null
    * @return Eigen::Affine3d = transfrom of echoed frame
    */
    Eigen::Affine3d getTrasform();

  }; // closing class

} // closing namespace

#endif // CONTACT_STATE_H
