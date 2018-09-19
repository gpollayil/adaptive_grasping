#ifndef CONTACT_STATE_H
#define CONTACT_STATE_H

#include <Eigen/Geometry>
#include <std_msgs/Int8.h>

/**
* @brief This class is called by the adaptive_grasping method to get the
* state of the contacts (i.e. how many and which fingers are touching). This
* means that it listens to a topic for collisions, saves the ids and the values
* of the touching fingers and its frames (all inside the class)
*/

class contactState {

public:

  // public typedefs ???
  //typedef boost::shared_ptr<IMUGraspControllerCaller> Ptr;
  //typedef boost::shared_ptr<const IMUGraspControllerCaller> ConstPtr;

  /** CONSTRUCTOR
  * @brief Default constructor for contactState
  *
  * @param null
  * @return null
  */
  contactState();

  /** DESTRUCTOR
  * @brief Default destructor for contactState
  *
  * @param null
  * @return null
  */
  ~contactState();

  /** HANDLECOLLISION
  * @brief Callback function to handle the touching topic (finger_col_sub)
  *
  * @param Int8 msg
  *   the listened message or the finger_col_sub
  * @return void (as all callbacks)
  */
  void handleCollision(const std_msgs::Int8::ConstPtr& msg);

private:

  // The finger which has just touched (read via topic)
  int touching_finger;

  // The vector containing all the fingers in collision
  std::map< int, Eigen::Affine3d > touched_fingers_details;

  // Subscribing to handle finger collision
  ros::Subscriber finger_col_sub;

}

#endif // CONTACT_STATE_H
