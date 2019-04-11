#include "matricesCreator.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "matrices_creator"
#define DEBUG             0   // print out additional info

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* DEFAULT CONSTRUCTOR */
matricesCreator::matricesCreator(){
    // Nothing to do here
}

/* CONSTRUCTOR */
matricesCreator::matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
  std::string palm_frame_name_, std::vector<int> joint_numbers_){
    // Initializing the object
    initialized = initialize(H_i_, world_frame_name_, palm_frame_name_, joint_numbers_);
}

/* DESTRUCTOR */
matricesCreator::~matricesCreator(){
  // Do nothing
}

/* INITIALIZE */
bool matricesCreator::initialize(Eigen::MatrixXd H_i_, std::string world_frame_name_,
  std::string palm_frame_name_, std::vector<int> joint_numbers_){
    // Set the basic contact selection matrix
    changeContactType(H_i_);

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

/* CHANGECONTACTTYPE */
void matricesCreator::changeContactType(Eigen::MatrixXd H_i_){
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
  // Set object pose
  object_pose = object_pose_;

  // Print message for debug
  if(DEBUG) std::cout << "Object pose set in matricesCreator!" << std::endl;
}

/* SETPERMUTATIONVECTOR */
void matricesCreator::setPermutationVector(Eigen::VectorXd p_vector_){
  // Set the permutation vector
  p_vector = p_vector_;

  // Print message for debug
  if(DEBUG) std::cout << "Permutation vector set in matricesCreator!" << std::endl;
}

/* SETOTHERPERMUTATIONSTUFF */
void matricesCreator::setOtherPermutationStuff(Eigen::VectorXd touch_indexes_){
  // Set the touch indexes vector
  touch_indexes = touch_indexes_;

  // Print message for debug
  if(DEBUG) std::cout << "The touch indexes vector set in matricesCreator!" << std::endl;
}

/* COMPLETEPEWHOLERMUTATIONVECTOR */
void matricesCreator::computeWholePermutationVector(Eigen::VectorXd p_vector_, int contacts_num_, Eigen::VectorXd touch_indexes_){
  // Checking if the touch_indexes_ vector complies with the dimension of H_i
  if (touch_indexes_.size() != H_i.rows()) {
    ROS_ERROR_STREAM("matricesCreator::computeWholePermutationVector : dimensions of touch_indexes_ != dimensions of H_i");
  }
  
  // Creating a tmp vector which will be the full permutation vector
  std::vector<double> temp_vector_full;

  // Now, iterating on p_vector_ to fill up correctly the full permutation vector
  int length_p_vec = p_vector_.size();
  for (int j = 0; j < length_p_vec; j++) {
    // Checking if ith elem of p_vector_ is in touch_indexes_
    bool is_in_touch_indexes = false;
    for (int k = 0; k < touch_indexes_.size(); k++) {
      if (p_vector_(j) == touch_indexes_(k)) is_in_touch_indexes = true;
    }

    // Pushing back the indexes according to is_in_touch_indexes
    if (is_in_touch_indexes && (contacts_num_ > 0)) {
      if(DEBUG || true) std::cout << "Starting to pushback touch_indexes stuff because is_in_touch_indexes is " << is_in_touch_indexes
        << " and contacts_num_ is " << contacts_num_ << std::endl;
      for (int con_it = 0; con_it < contacts_num_; con_it++) {
        temp_vector_full.push_back(p_vector_(j) + con_it*touch_indexes_.size());
      }
    } else {
      if(!is_in_touch_indexes) temp_vector_full.push_back(p_vector_(j));
    }
  }

  Eigen::VectorXd temp_eig = Eigen::VectorXd::Map(temp_vector_full.data(), temp_vector_full.size());
  
  if (DEBUG || true) ROS_INFO_STREAM("matricesCreator::computeWholePermutationVector : the computed whole perm. vector is \n " << temp_eig);

  this->p_vector_full = temp_eig;
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

  // Print message for debug
  if(DEBUG) std::cout << "Finished KDL setup in matricesCreator!" << std::endl;

  return true;
}

/* COMPUTEALLMATRICES */
void matricesCreator::computeAllMatrices(){
  // Compute matrices J, G, T and H
  computeWholeGrasp(contacts_map);
  computeWholePoleChange(contacts_map);
  computeWholeContactSelection(contacts_map);
  computeWholeJacobian(contacts_map, joints_map);
  computeWholePermutationVector(p_vector, contacts_map.size(), touch_indexes);
  computePermutationMatrix(this->p_vector_full, contacts_map.size());

  // Print message for debug
  if(DEBUG) std::cout << "Finished computing in matricesCreator!" << std::endl;
  ROS_DEBUG_STREAM("Finished computing in matricesCreator!");
}

/* READALLMATRICES */
void matricesCreator::readAllMatrices(Eigen::MatrixXd& read_J,
  Eigen::MatrixXd& read_G, Eigen::MatrixXd& read_T, Eigen::MatrixXd& read_H, Eigen::MatrixXd& read_P){
    read_J = J; read_G = G; read_T = T; read_H = H; read_P = P;
}

/* COMPUTEJACOBIAN */
KDL::Jacobian matricesCreator::computeJacobian(KDL::Chain chain,
  KDL::JntArray q_){
    // Reset the jacobian solver
    jacobian_solver.reset(new KDL::ChainJntToJacSolver(chain));

    // Compute jacobian for q_
    KDL::Jacobian Ja_i;
    Ja_i.resize(chain.getNrOfJoints());     // This resizing is important!!!
    jacobian_solver->JntToJac(q_, Ja_i);    // Without resizing this won't work

    // Printing eventually the error code
    int error_code = jacobian_solver->getError();
    if(DEBUG) std::cout << "computeJacobian: In computing Jacobian: " <<
      "the error code was " << jacobian_solver->strError(error_code) <<
        "." << std::endl;

    // Print message for debug
    if(DEBUG){
      std::cout << "Ja_i =" << std::endl;
      std::cout << Ja_i.data << std::endl;
    }

    // Return result
    return Ja_i;
}

/* TRANSFORMJACOBIAN */
Eigen::MatrixXd matricesCreator::transformJacobian(KDL::Jacobian Jac){
  // Debug message
  if(DEBUG) std::cout << "transformJacobian: just entered!" << std::endl;

  // Compute the needed matrices for base change
  Eigen::MatrixXd R_p_w = Palm_to_World.inverse().rotation();
  Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);

  // Debug message
  if(DEBUG) std::cout << "transformJacobian: computed matrices!" << std::endl;

  // Compute the jacobian change of base matrix
  Eigen::MatrixXd M_p_w(6, 6);
  M_p_w << R_p_w, O_3, O_3, R_p_w;

  // Print matrices
  if(DEBUG) std::cout << "transformJacobian: Jac transf matrix: " << std::endl;
  if(DEBUG) std::cout << M_p_w << std::endl;
  if(DEBUG) std::cout << "transformJacobian: the Jac matrix: " << std::endl;
  if(DEBUG) std::cout << Jac.data << std::endl;

  // Perform the change of base and return
  return M_p_w * Jac.data;
}

/* COMPUTEGRASP */
Eigen::MatrixXd matricesCreator::computeGrasp(Eigen::Affine3d contact_pose,
  Eigen::Affine3d object_pose_){
    // Getting object-contact vector
    Eigen::Vector3d OC =
      contact_pose.translation() - object_pose_.translation();

    // For debugging
    ROS_DEBUG_STREAM("Contact translation: \n" << contact_pose.translation() << ".");
    ROS_DEBUG_STREAM("Object translation: \n" << object_pose_.translation() << ".");
    ROS_DEBUG_STREAM("So OC is: \n" << OC << ".");

    // Creating skew matrix
    Eigen::Matrix3d OC_hat;
    OC_hat << 0, -OC(2), OC(1), OC(2), 0, -OC(0), -OC(1), OC(0), 0;

    // Blockwise building the grasp matrix
    Eigen::MatrixXd G_i; G_i.resize(6, 6);
    Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);
    G_i << I_3, O_3 - OC_hat, O_3, I_3;
    /* CLARIFICATION: the above is the transpose of what is usually defined as
    a grasp matrix G */

    // Print message for debug
    if(DEBUG){
      std::cout << "Gi =" << std::endl;
      std::cout << G_i.transpose() << std::endl;
    }

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
    Palm_to_World = P_to_W;   // Stored for transformJacobian

    // Transform it into world coordinates
    Eigen::Vector3d PC_w = P_to_W.inverse().rotation() * PC_p;

    // For debugging
    ROS_DEBUG_STREAM("PC in P is: \n" << PC_p << ".");
    ROS_DEBUG_STREAM("PC in W is: \n" << PC_w << ".");
    ROS_DEBUG_STREAM("pwp is: \n" << P_to_W.translation() << ".");
    ROS_DEBUG_STREAM("Rwp is: \n" << P_to_W.rotation() << ".");

    // Creating the skew matrix
    Eigen::Matrix3d PC_hat;
    PC_hat << 0, -PC_w(2), PC_w(1), PC_w(2), 0, -PC_w(0), -PC_w(1), PC_w(0), 0;

    // Blockwise building the grasp matrix
    Eigen::MatrixXd T_i; T_i.resize(6, 6);
    Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);
    T_i << I_3, O_3 - PC_hat, O_3, I_3;

    // Print message for debug
    if(DEBUG){
      std::cout << "T_i =" << std::endl;
      std::cout << T_i << std::endl;
    }

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
    J = Eigen::MatrixXd::Zero(6 * contacts_map_.size(), total_joints);

    // Index to go right blockwise on J
    int k = 0;

    Eigen::MatrixXd J_i_temp;

    // Printing out the contacts map
    if(DEBUG){
      std::cout << "Contacts map in matricesCreator is:" << std::endl;
      for(auto elem : contacts_map_){
        std::cout << elem.first << " : " << std::get<0>(elem.second) << "." << std::endl;
      }
    }

    // For each contact, compute J_i and compose into J
    for(it_c = contacts_map_.begin(); it_c != contacts_map_.end(); ++it_c){
      // Getting the current finger
      int current_finger = it_c->first;

      // Creating the finger's kinematic chain from the tree
      robot_kin_tree.getChain(palm_frame_name, std::get<0>(it_c->second),
        finger_kin_chain);

      if(DEBUG){
        std::cout << "The segments of current chain are:" << std::endl;
        for(auto it : finger_kin_chain.segments){
          std::cout << it.getName() << std::endl;
        }
        std::cout << "The chain has " << finger_kin_chain.getNrOfJoints() << " joints." << std::endl;
      }

      // Printing current finger kinematic chain
      if(DEBUG) std::cout << "Kinematic chain has: " <<
        finger_kin_chain.getNrOfJoints() << " no. of joints." << std::endl;

      // Get the joint array for the considered finger
      finger_joint_array = getFingerJoints(joints_map, current_finger);

      // Printing current finger joint array
      if(DEBUG) std::cout << "Finger joint array is: " <<
        finger_joint_array.data << std::endl;

      // Get the jacobian for the current chain in the obtained jntarray
      KDL::Jacobian J_i = computeJacobian(finger_kin_chain, finger_joint_array);

      // Print message for debug
      if(DEBUG) std::cout << "Computing the horizontal position for J_i in J"
        " in matricesCreator!" << std::endl;

      // Find out the horizontal block position of J_i in J
      int h = 0;
      for(int z = 0; z <= (current_finger-1); z++) h += joint_numbers[z];
      h = h - joint_numbers[current_finger-1];

      // Print message for debug
      if(DEBUG) std::cout << "Put J_i in J in matricesCreator!" << std::endl;
      if(DEBUG) std::cout << "Index k = " << k << "." << std::endl;
      if(DEBUG) std::cout << "Index h = " << h << "." << std::endl;

      // Now, put the current jacobian into the whole Jacobian matrix
      J_i_temp = transformJacobian(J_i);
      // J_i_temp = J_i.data;

      // Print Eigen and a message for debug
      if(DEBUG) std::cout << "The current finger is " << std::get<0>(it_c->second) << "." << std::endl;
      if(DEBUG) std::cout << "J_i (in palm frame) is: " << std::endl;
      if(DEBUG) std::cout << J_i.data << std::endl;
      if(DEBUG) std::cout << "J_i (in world frame) is: " << std::endl;
      if(DEBUG) std::cout << J_i_temp << std::endl;
      if(DEBUG) std::cout << "KDL to Eigen in matricesCreator!" << std::endl;

      if(current_finger == 1) J.block<6, 5>(k, h) = J_i_temp;
      else J.block<6, 7>(k, h) = J_i_temp;

      // Print message for debug
      if(DEBUG) std::cout << "Done J_i in J in matricesCreator!" << std::endl;

      // Increment the index k to go to next block
      k += 6;
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

    // Index to go right blockwise on G
    int k = 0;

    // For each contact, compute G_i and compose into G
    for(it_c = contacts_map_.begin(); it_c != contacts_map_.end(); ++it_c){
      // Compute the grasp matrix for the current contact
      Eigen::MatrixXd G_i = computeGrasp(std::get<1>(it_c->second),
        object_pose);

      // Now, put the current grasp matrix into the whole grasp matrix
      G.block<6, 6>(0, k) = G_i;

      // Increment the index k to go to next block
      k += 6;
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

    // Index to go down blockwise on T
    int k = 0;

    // For each contact, compute T_i and compose into T
    for(it_c = contacts_map_.begin(); it_c != contacts_map_.end(); ++it_c){
      // Compute the twist pole change matrix from palm to current contact
      Eigen::MatrixXd T_i = computePoleChange(std::get<1>(it_c->second),
        std::get<2>(it_c->second));

      // Now, put the current pole change into the whole pole change matrix
      T.block<6, 6>(k, 0) = T_i;

      // Increment the index k to go to next block
      k += 6;
    }
}

/* COMPUTEWHOLECONTACTSELECTION */
void matricesCreator::computeWholeContactSelection(std::map<int,
  std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>> contacts_map_){
    // Creating an iterator for contacts_map
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>>::iterator it_c;

    // Resize and set to null the whole contact selection MatrixXd
    H = Eigen::MatrixXd::Zero(H_i.rows() * contacts_map_.size(),
      6 * contacts_map_.size());

    // Indexes to put H_i in diagonal positions
    int k = 0;
    int h = 0;

    // For each contact, compute H_i (in world frame) and compose into H
    for(it_c = contacts_map_.begin(); it_c != contacts_map_.end(); ++it_c){
      // Compute the world to finger transform for H_i
      Eigen::MatrixXd M_i(6, 6);
      Eigen::MatrixXd R_i = std::get<1>(it_c->second).rotation();
      Eigen::MatrixXd O_3 = Eigen::MatrixXd::Zero(3, 3);
      M_i << R_i, O_3, O_3, R_i;

      // Get the contact selection in world frames
      Eigen::MatrixXd H_i_w = H_i * M_i;

      // Now, put the current contact selection into the whole diag. H matrix
      H.block(k, h, H_i.rows(), 6) = H_i_w;

      // Increment the indexes to shift through diagonal of H
      k += H_i.rows();
      h += 6;
    }
}

/* COMPUTEPERMUTATIONMATRIX */
void matricesCreator::computePermutationMatrix(Eigen::VectorXd p_vector_, int contacts_num_){
  // TODO : contacts_num_ is unused, change it everywhere
  
  // Getting the size of the permutation vector
  int length_p = p_vector_.size();
  if(DEBUG) ROS_INFO_STREAM("matricesCreator::computePermutationMatrix The length of the permutation vector is " << 
    length_p << " because it is " << p_vector_);

  // Creating permutation matrix from p_vector_
  Eigen::MatrixXd P_temp = Eigen::MatrixXd::Zero(length_p, length_p);
  for(int i = 0; i < length_p; i++){
    P_temp(i, p_vector_(i) - 1) = 1;
  }

  if(DEBUG || true) ROS_INFO_STREAM("matricesCreator::computePermutationMatrix The total permutation matrix: \n" << P_temp << ".");

  // Setting the permutation matrix
  P = P_temp;

}
