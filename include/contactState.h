#ifndef CONTACT_STATE_H
#define CONTACT_STATE_H

#include <map>
#include <tuple>
#include <Eigen/Geometry>
#include <std_msgs/Int8.h>
#include <ros/ros.h>

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
    * @param null
    * @return null
    */
    contactState(std::string topic_name);

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

    // The finger which has just touched (read via topic)
    int touching_finger;

    // The vector containing info on all the fingers in collision
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map;

    // Temporary string for saving parameters
    std::string temp_param;

    // Map for storing already read parameters
    std::map<std::string, std::string> params_map;

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

    /** GETTRANSFORMS
    * @brief Class function to get frames for the touching fingers
    *   which will be written into touched_fingers_details
    *
    * @param null
    * @return void
    */
    void getTrasforms();

  }; // closing class

} // closing namespace

#endif // CONTACT_STATE_H
