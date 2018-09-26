#include "matricesCreator.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "matrices_creator"
#define DEBUG             1   // print out additional info

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
matricesCreator::matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
  std::string palm_frame_name_, std::vector<int> joint_numbers_){
    // Set the basic contact selection matrix
    changeHandType(H_i_);

    // Set the frame names for world and palm
    changeFrameNames(world_frame_name_, palm_frame_name_);

    // Set number of joints of the robotic hand (total and for each finger)
    joint_numbers = joint_numbers_;
    total_joints = 0;
    for(int i : joint_numbers_) total_joints += i;

    // Prepare KDL to get the robot kinematic tree
    prepareKDL();

    // Print message for debug
    if(DEBUG) std::cout << "Built matricesCreator!" << std::endl;
}

/* DESTRUCTOR */
matricesCreator::~matricesCreator(){
  // Do nothing
}

/* CHANGEHANDTYPE */
void matricesCreator::changeHandType(Eigen::MatrixXd H_i_){
  // Set the new basic contact selection matrix
  H_i = H_i_;

  // Print message for debug
  if(DEBUG) std::cout << "Changed hand type in matricesCreator!" << std::endl;
}

/* CHANGEFRAMENAMES */
void matricesCreator::changeFrameNames(std::string world_frame_name_,
  std::string palm_frame_name_){
    // Set the new frame names for world and palm
    world_frame_name = world_frame_name_; palm_frame_name = palm_frame_name_;

    // Print message for debug
    if(DEBUG) std::cout << "Changed frames in matricesCreator!" << std::endl;
}

/* SETCONTACTSMAP */
void matricesCreator::setContactsMap(std::map<int, std::tuple<std::string,
  Eigen::Affine3d, Eigen::Affine3d>> contacts_map_){
    // Set the contacts map in the relative private variable
    contacts_map = contacts_map_;

    // Print message for debug
    if(DEBUG) std::cout << "Contacts map set in matricesCreator!" << std::endl;
}

/* SETJOINTSMAP */
void matricesCreator::setJointsMap(std::map<int,
  sensor_msgs::JointState> joints_map_){
    // Set the joint states map in the relative private variable
    joints_map = joints_map_;

    // Print message for debug
    if(DEBUG) std::cout << "Joints map set in matricesCreator!" << std::endl;
}

/* SETOBJECTPOSE */
void matricesCreator::setObjectPose(Eigen::Affine3d object_pose_){
  object_pose = object_pose_;

  // Print message for debug
  if(DEBUG) std::cout << "Object pose set in matricesCreator!" << std::endl;
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

  // Print message for debug
  if(DEBUG) std::cout << "Finished KDL setup in matricesCreator!" << std::endl;
}

/* COMPUTEALLMATRICES */
void matricesCreator::computeAllMatrices(){
  // Compute matrices J, G, T and H
  computeWholeJacobian(contacts_map, joints_map);
  computeWholeGrasp(contacts_map);
  computeWholePoleChange(contacts_map);
  computeWholeContactSelection(contacts_map);

  // Print message for debug
  if(DEBUG) std::cout << "Finished computing in matricesCreator!" << std::endl;
}

/* READALLMATRICES */
void matricesCreator::readAllMatrices(Eigen::MatrixXd& read_J,
  Eigen::MatrixXd& read_G, Eigen::MatrixXd& read_T, Eigen::MatrixXd& read_H){
    read_J = J; read_G = G; read_T = T; read_H = H;
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
  Eigen::Affine3d object_pose_){
    // Getting object-contact vector
    Eigen::Vector3d OC =
      object_pose_.translation() - contact_pose.translation();

    // Creating skew matrix
    Eigen::Matrix3d OC_hat;
    OC_hat << 0, -OC(2), OC(1), OC(2), 0, -OC(0), -OC(1), OC(0), 0;

    // Blockwise building the grasp matrix
    Eigen::MatrixXd G_i; G_i.resize(6, 6);
    Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);
    G_i << I_3, O_3 - OC_hat, O_3, I_3;

    // Return the result
    return G_i.transpose();
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
      for(int i = 1; i <= 5; i++){
        if(i == current_finger){
          // Insert J_i in correct position
          J_i_row << J_i.data;
        } else {
          // Put zeros everywhere else
          J_i_row << Eigen::MatrixXd::Zero(6, joint_numbers[i-1]);
        }
      }

      // Now, put the current finger's jacobian row into the whole jacobian
      J << J_i_row;
    }
}

/* COMPUTEWHOLEGRASP */
void matricesCreator::computeWholeGrasp(std::map<int, std::tuple<std::string,
  Eigen::Affine3d, Eigen::Affine3d>> contacts_map_){
    // Creating an iterator for contacts_map
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>>::iterator it_c;

    // Resize the whole grasp MatrixXd
    G.resize(6, 6 * contacts_map_.size());

    // For each contact, compute G_i and compose into G
    for(it_c = contacts_map.begin(); it_c != contacts_map.end(); ++it_c){
      // Compute the grasp matrix for the current contact
      Eigen::MatrixXd G_i = computeGrasp(std::get<1>(it_c->second),
        object_pose);

      // Now, put the current grasp matrix into the whole grasp matrix
      G << G_i;
    }
}

/* COMPUTEWHOLEPOLECHANGE */
void matricesCreator::computeWholePoleChange(std::map<int,
  std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>> contacts_map_){
    // Creating an iterator for contacts_map
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>>::iterator it_c;

    // Resize the whole pole change MatrixXd
    T.resize(6 * contacts_map_.size(), 6);

    // For each contact, compute T_i and compose into T
    for(it_c = contacts_map.begin(); it_c != contacts_map.end(); ++it_c){
      // Compute the twist pole change matrix from palm to current contact
      Eigen::MatrixXd T_i = computePoleChange(std::get<1>(it_c->second),
        std::get<2>(it_c->second));

      // Now, put the current pole change into the whole pole change matrix
      T << T_i;
    }
}

/* COMPUTEWHOLECONTACTSELECTION */
void matricesCreator::computeWholeContactSelection(std::map<int,
  std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>> contacts_map_){
    // Creating an iterator for contacts_map
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>>::iterator it_c;

    // Resize the whole contact selection MatrixXd
    H.resize(6 * contacts_map_.size(), 6 * contacts_map_.size());

    // Index to put H_i in diagonal positions
    int k = 1;

    // For each contact, compute H_i (in world frame) and compose into H
    for(it_c = contacts_map.begin(); it_c != contacts_map.end(); ++it_c){
      // Compute the local to world transform for H_i
      Eigen::MatrixXd M_i(6, 6);
      Eigen::MatrixXd R_i = std::get<1>(it_c->second).linear();
      Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
      M_i << R_i, O_3, R_i, O_3;

      // Get the contact selection in world frames
      Eigen::MatrixXd H_i_w = H_i * M_i;

      // Compute the current row of the whole contact selection matrix
      Eigen::MatrixXd H_i_row(H_i_w.rows(), 6 * contacts_map_.size());

      for(int i = 1; i <= contacts_map_.size(); i++){
        if(i == k){
          // Insert J_i in correct position
          H_i_row << H_i_w;
        } else {
          // Put zeros everywhere else
          H_i_row << Eigen::MatrixXd::Zero(H_i_w.rows(), 6);
        }
      }

      // Increment the index to shift through diagonal of H
      k++;

      // Now, put the current contact selection row into the whole contact sel.
      H << H_i_row;
    }
}
