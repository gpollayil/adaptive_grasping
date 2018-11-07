#include "contactState.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "contact_state"
#define DEBUG             0                       // prints out additional info

/**
* @brief The following are functions of the class contactState.
*
*/

using namespace adaptive_grasping;

/* DEFAULT CONSTRUCTOR */
contactState::contactState(){
    // Nothing to do here
}

/* CONSTRUCTOR */
contactState::contactState(std::string topic_name,
  std::map<int, std::string> link_names_map_,
    std::map<std::string, std::string> params_map_){
    // Initialize the object
    this->initialized = intialize(topic_name, link_names_map_, params_map_);
}

/* DESTRUCTOR */
contactState::~contactState(){
  // Nothing to do here
}

/* READVALUES */
void contactState::readValues(std::map<int, std::tuple<std::string,
  Eigen::Affine3d, Eigen::Affine3d>>& input_map_,
    std::map<int, sensor_msgs::JointState>& input_map2_){
    // Reading the contacts_map
    contact_state_mutex.lock();                             // mutex on
    input_map_ = contacts_map;
    input_map2_ = joints_map;
    contact_state_mutex.unlock();                           // mutex off
}

/* INITIALIZE */
bool contactState::intialize(std::string topic_name,
  std::map<int, std::string> link_names_map_,
    std::map<std::string, std::string> params_map_){
    // The finger_id is 0 as there are no contacts yet
    touching_finger = 0;

    // Subscribing to topic (queue is 1 for quick processing)
    finger_col_sub = node_contact_state.subscribe(topic_name, 1,
      &contactState::handleCollision, this);

    // Initializing the service client
    fj_client = node_contact_state.serviceClient<finger_fk::FingerJointsService>("fj_service");

    // Constructing the maps
    this->params_map = params_map_;
    this->link_names_map = link_names_map_;
}

/* HANDLECOLLISION */
void contactState::handleCollision(const std_msgs::Int8::ConstPtr& msg){
  // This if-else is needed to avoid the zeros in touching finger topic to create
  // problems in the contacts and joints maps updating
  if(msg->data == 0){
    if(DEBUG) ROS_INFO("There have been no touches yet! adaptive_grasping break!");
  } else {
    // Temporarily saving message
    touching_finger = msg->data;
  }

  // This big if is needed to not updating the maps while there is no contact yet
  if(touching_finger != 0){
    // Checking if the echoed finger id is a good value
    std::map<int, std::string>::iterator it_names = link_names_map.find(touching_finger);
    if(it_names == link_names_map.end()){
      ROS_ERROR("THE ECHOED ID IS NOT GOOD: IT SHOULD HAVE BEEN BETWEEN 1 AND 5");
    }

    // Creating a tuple with identity Affine3ds
    Eigen::Affine3d identity_aff = Eigen::Affine3d::Identity();
    std::string touched_link_name = link_names_map.at(touching_finger);
    if(DEBUG) std::cout << "The touching finger link is " << touched_link_name << "." << std::endl;
    std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d> empty_tuple (std::make_tuple(touched_link_name,
        identity_aff, identity_aff));
    if(DEBUG) std::cout << "The touching finger link in empty_tuple is " << std::get<0>(empty_tuple) << "." << std::endl;
    
    // Analogously creating an empty joint state
    sensor_msgs::JointState empty_joints;

    // Just to be sure that the last touched finger id is in the map, inserting
    // (eventually overwriting) the element in the contacts_map
    contact_state_mutex.lock();                             // mutex on
    contacts_map[touching_finger] = empty_tuple;
    joints_map[touching_finger] = empty_joints;
    contact_state_mutex.unlock();                           // mutex off

    // Iteratively updating contacts_map and joints_map
    iterateContacts();
    iterateJoints();
  }

  // Printing out the contacts map
  if(DEBUG){
    std::cout << "Contacts map in contactState is:" << std::endl;
    for(auto elem : contacts_map){
      std::cout << elem.first << " : " << std::get<0>(elem.second) << "." << std::endl;
    }
  }
}

/* ITERATECONTACTS */
void contactState::iterateContacts(){
  // Creating an iterator for contacts_map
  // std::map<int, std::tuple<std::string, Eigen::Affine3d,
  //   Eigen::Affine3d>>::iterator it_c;

    // Now with a loop echoing and saving all needed transforms in contacts_map
    for(auto it_c : contacts_map){
      // Getting the frame names
      std::string frame_fing = std::get<0>(it_c.second);
      if(DEBUG) std::cout << "Managing contacts for " << frame_fing << "." << std::endl;
      std::string frame_world = params_map.at("world_name");
      std::string frame_palm = params_map.at("palm_name");

      // Getting all the needed transforms
      Eigen::Affine3d fing_aff = getTrasform(frame_world, frame_fing);
      Eigen::Affine3d palm_aff = getTrasform(frame_palm, frame_fing);
      // CLARIFICATION: The transformation palm_aff is from palm to finger

      // Writing the correct tuple into the map
      std::tuple<std::string, Eigen::Affine3d,
        Eigen::Affine3d> correct_tuple (std::make_tuple(frame_fing,
          fing_aff, palm_aff));
      contact_state_mutex.lock();                             // mutex on
      contacts_map[it_c.first] = correct_tuple;
      contact_state_mutex.unlock();                           // mutex off
    }
}

/* ITERATEJOINTS */
void contactState::iterateJoints(){
  // Creating an iterator for joints_map
  std::map<int, sensor_msgs::JointState>::iterator it_j;

  // Now with another loop echoing and saving all needed joints in joints_map
  for(it_j = joints_map.begin(); it_j != joints_map.end(); ++it_j){
    // Getting the joint state from service
    sensor_msgs::JointState correct_joints;

    // Creating an srv with touching_finger id and filling up
    if(DEBUG) ROS_INFO("The Finger Joint srv is being filled!");
    finger_fk::FingerJointsService srv;
    srv.request.finger_id = it_j->first;

    // Calling the service
    if (fj_client.call(srv)) {
      if(DEBUG){
        ROS_INFO("The result is as follows:");
        for(size_t i = 0; i < srv.response.joint_state.name.size(); i++) {
          std::cout << i << ", " << srv.response.joint_state.name[i] << " : " <<
            srv.response.joint_state.position[i] << ";" << std::endl;
        }
      }
    } else {
      ROS_ERROR("Failed to call finger_joints_service");
    }

    correct_joints = srv.response.joint_state;

    // Writing the correct JointState into the map
    contact_state_mutex.lock();                             // mutex on
    joints_map[it_j->first] = correct_joints;
    contact_state_mutex.unlock();                           // mutex off
  }
}

/* GETTRANSFORM*/
Eigen::Affine3d contactState::getTrasform(std::string frame1_name,
  std::string frame2_name){
    // tf echoing using input frame names
    try {
		tf_listener.waitForTransform(std::string("/") + frame1_name,
      std::string("/") + frame2_name, ros::Time(0), ros::Duration(1.0) );
		tf_listener.lookupTransform(std::string("/") + frame1_name,
      std::string("/") + frame2_name, ros::Time(0), stamped_transform);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Converting to Affine3d
    Eigen::Affine3d affine;
    tf::Transform transform(stamped_transform.getRotation(),
      stamped_transform.getOrigin());
    tf::transformTFToEigen(transform, affine);

    return affine;
}
