#include "contactState.h"

/**
* @brief The following are functions of the class contactState.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
contactState::contactState(std::string topic_name,
  std::map<int, std::string> link_names_map_,
    std::map<std::string, std::string> params_map_){
  // Subscribing to topic
  finger_col_sub = node_contact_state.subscribe(topic_name, 1000,
    &contactState::handleCollision, this);

  // Initializing the service client
  fj_client = node_contact_state.serviceClient<finger_fk::FingerJointsService>("fj_service");

  // Constructing the maps
  this->params_map = params_map_;
  this->link_names_map = link_names_map_;
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

/* HANDLECOLLISION */
void contactState::handleCollision(const std_msgs::Int8::ConstPtr& msg){
  // Temporary saving of message
  touching_finger = msg->data;

  // Creating an tuple with identity Affine3ds
  Eigen::Affine3d identity_aff = Eigen::Affine3d::Identity();
  std::string touched_link_name = link_names_map.at(touching_finger);
  std::tuple<std::string, Eigen::Affine3d,
    Eigen::Affine3d> empty_tuple (std::make_tuple(touched_link_name,
      identity_aff, identity_aff));

  // Analogously creating an empty joint state
  sensor_msgs::JointState empty_joints;

  // Just to be sure that the last touched finger id is in the map, inserting
  // (eventually overwriting) the element in the contacts_map
  contact_state_mutex.lock();                             // mutex on
  contacts_map[touching_finger] = empty_tuple;
  joints_map[touching_finger] = empty_joints;
  contact_state_mutex.unlock();                           // mutex off

  // Creating an iterator for contacts_map and for joints_map
  std::map<int, std::tuple<std::string, Eigen::Affine3d,
    Eigen::Affine3d>>::iterator it_c;

  std::map<int, sensor_msgs::JointState>::iterator it_j;

  // Now with a loop echoing and saving all needed transforms in contacts_map
  for(it_c = contacts_map.begin(); it_c != contacts_map.end(); ++it_c){
    // Getting the frame names
    std::string frame_fing = std::get<0>(it_c->second);
    std::string frame_world = params_map.at("world_name");
    std::string frame_palm = params_map.at("palm_name");

    // Getting all the needed transforms
    Eigen::Affine3d fing_aff = getTrasform(frame_world, frame_fing);
    Eigen::Affine3d palm_aff = getTrasform(frame_fing, frame_palm);

    // Writing the correct tuple into the map
    std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d> correct_tuple (std::make_tuple(frame_fing,
        fing_aff, palm_aff));
    contact_state_mutex.lock();                             // mutex on
    contacts_map[touching_finger] = correct_tuple;
    contact_state_mutex.unlock();                           // mutex off

  }

  // Now with another loop echoing and saving all needed joints in joints_map
  for(it_j = joints_map.begin(); it_j != joints_map.end(); ++it_j){
    // Getting the joint state from service
    sensor_msgs::JointState correct_joints;

    // Creating an srv with touching_finger id and filling up
    if(DEBUG) ROS_INFO("The Finger Joint srv is being filled!");
    finger_fk::FingerJointsService srv;
    srv.request.finger_id = touching_finger;

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
    joints_map[touching_finger] = correct_joints;
    contact_state_mutex.unlock();                           // mutex off

  }
}

/* GETTRANSFORM*/
Eigen::Affine3d contactState::getTrasform(std::string frame1_name,
  std::string frame2_name){
    // tf echoing using input frame names
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_transform;
    try {
		tf_listener.waitForTransform(std::string("/") + frame1_name,
      std::string("/") + frame2_name, ros::Time(0), ros::Duration(10.0) );
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
