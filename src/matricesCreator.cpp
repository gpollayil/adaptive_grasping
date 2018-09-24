#include "matricesCreator.h"

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
matricesCreator::matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
  std::string palm_frame_name_, unsigned int total_joints_){
    // Set the basic contact selection matrix
    changeHandType(H_i_);

    // Set the frame names for world and palm
    changeFrameNames(world_frame_name_, palm_frame_name_);

    // Set number of joints of the robotic hand
    total_joints = total_joints_;

    // Prepare KDL to get the robot kinematic tree
    prepareKDL();
}

/* DESTRUCTOR */
matricesCreator::~matricesCreator(){
  // Do nothing
}

/* CHANGEHANDTYPE */
void matricesCreator::changeHandType(Eigen::MatrixXd H_i_){
  // Set the new basic contact selection matrix
  H_i = H_i_;
}

/* CHANGEFRAMENAMES */
void matricesCreator::changeFrameNames(std::string world_frame_name_,
  std::string palm_frame_name_){
    // Set the new frame names for world and palm
    world_frame_name = world_frame_name_; palm_frame_name = palm_frame_name_;
}

/* SETCONTACTSMAP */
void matricesCreator::setContactsMap(std::map<int, std::tuple<std::string,
  Eigen::Affine3d, Eigen::Affine3d>> contacts_map_){
    // Set the contacts map in the relative private variable
    contacts_map = contacts_map_;
}

/* SETJOINTSMAP */
void matricesCreator::setJointsMap(std::map<int,
  sensor_msgs::JointState> joints_map_){
    // Set the joint states map in the relative private variable
    joints_map = joints_map_;
}

/* SETOBJECTPOSE */
void matricesCreator::setObjectPose(Eigen::Affine3d object_pose_){
  object_pose = object_pose_;
}

/* PREPAREKDL */
bool matricesCreator::prepareKDL(){
  // Load robot description from ROS parameter server
  std::string robot_description_string;
  nh.param("robot_description", robot_description_string, std::string());

  // Get kinematic tree from robot description
  if (!kdl_parser::treeFromString(robot_description_string, robot_kin_tree)){
    ROS_ERROR("Failed to get robot kinematic tree!");
    return false;
  }
  return true;
}

/* COMPUTEJACOBIAN */
KDL::Jacobian matricesCreator::computeJacobian(KDL::Chain chain,
  KDL::JntArray q_){
    // Reset the jacobian solver
    jacobian_solver.reset(new KDL::ChainJntToJacSolver(chain));

    // Compute jacobian for q_
    KDL::Jacobian J_i;
    jacobian_solver->JntToJac(q_, J_i);

    // Return result
    return J_i;
}

/* COMPUTEGRASP */
Eigen::MatrixXd matricesCreator::computeGrasp(Eigen::Affine3d contact_pose,
  Eigen::Affine3d object_pose){
    // Getting object-contact vector
    Eigen::Vector3d OC =
      object_pose.translation() - contact_pose.translation();

    // Creating skew matrix
    Eigen::Matrix3d OC_hat;
    OC_hat << 0, -OC(2), OC(1), OC(2), 0, -OC(0), -OC(1), OC(0), 0;

    // Blockwise building the grasp matrix
    Eigen::MatrixXd G_i; G_i.resize(6, 6);
    Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);
    G_i << I_3, O_3 - OC_hat, O_3, I_3;

    // Return the result
    return G_i;
}

/* COMPUTEPOLECHANGE */
Eigen::MatrixXd matricesCreator::computePoleChange(Eigen::Affine3d contact_pose,
  Eigen::Affine3d pc_pose){
    // Get the translation part of palm-contact transform
    Eigen::Vector3d PC_p = pc_pose.translation();

    // Get the palm to world transformation
    Eigen::Affine3d P_to_W = (contact_pose * pc_pose.inverse()).inverse();

    // Transform it into world coordinates
    Eigen::Vector3d PC_w = P_to_W.linear() * PC_p;

    // Creating the skew matrix
    Eigen::Matrix3d PC_hat;
    PC_hat << 0, -PC_w(2), PC_w(1), PC_w(2), 0, -PC_w(0), -PC_w(1), PC_w(0), 0;

    // Blockwise building the grasp matrix
    Eigen::MatrixXd T_i; T_i.resize(6, 6);
    Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);
    T_i << I_3, O_3 - PC_hat, O_3, I_3;

    // Return the result
    return T_i;
}

/* GETFINGERJOINTS */
KDL::JntArray matricesCreator::getFingerJoints(std::map<int,
  sensor_msgs::JointState> joints_map_, int finger_id_){
    // Get the finger joint state from the map
    auto finder = joints_map_.find(finger_id_);
    if(finder == joints_map_.end()){
      // key not found
      std::cerr << "Something went wrong! Coulding find joint states for "
        "finger with id: " << finger_id_ << "!" << '\n';
    }
    sensor_msgs::JointState finger_state = finder->second;

    // Create a joint array to convert into
    auto q_length = finger_state.position.size();
    KDL::JntArray temp_q = KDL::JntArray(q_length);

    // Insert the values from sensor_msgs::JointState to KDL::JntArray
    for(unsigned int i = 0; i < q_length; i++){
        temp_q(i) = (double) finger_state.position[i];
    }

    // Finally return the resulting joint array
    return temp_q;
}

/* COMPUTEWHOLEJACOBIAN */
void matricesCreator::computeWholeJacobian(std::map<int,
  std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>> contacts_map_,
  std::map<int, sensor_msgs::JointState> joints_map_){
    // Creating an iterator for contacts_map
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>>::iterator it_c;

    // Resize the whole jacobian MatrixXd
    J.resize(6 * contacts_map_.size(), total_joints);

    // For each contact, compute J_i and compose into J
    for(it_c = contacts_map.begin(); it_c != contacts_map.end(); ++it_c){
      // Getting the current finger
      int current_finger = it_c->first;

      // Creating the finger's kinematic chain from the tree
      robot_kin_tree.getChain(palm_frame_name, std::get<0>(it_c->second),
        finger_kin_chain);

      // Get the joint array for the considered finger
      finger_joint_array = getFingerJoints(joints_map, current_finger);

      // Get the jacobian for the current chain in the obtained jntarray
      KDL::Jacobian J_i = computeJacobian(finger_kin_chain, finger_joint_array);

      // Compute the current row of the whole jacobian
      Eigen::MatrixXd J_i_row(6, total_joints);

      // For loop to create the row
    }
}
