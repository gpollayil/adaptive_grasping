#include "matricesCreator.h"

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
matricesCreator::matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
  std::string palm_frame_name_){
    // Set the basic contact selection matrix
    changeHandType(H_i_);

    // Set the frame names for world and palm
    changeFrameNames(world_frame_name_, palm_frame_name_);

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
    contacts_map = contacts_map_;
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

/* COMPUTE JACOBIAN */
KDL::Jacobian matricesCreator::computeJacobian(KDL::Chain chain,
  KDL::JntArray q_){
    // Reset the jacobian solver
    jacobian_solver.reset(new KDL::ChainJntToJacSolver(chain));

    // Compute jacobian for q_
    KDL::Jacobian J_;
    jacobian_solver->JntToJac(q_, J_);

    // Return result
    return J_;
}

/* COMPUTE GRASP */
Eigen::MatrixXd matricesCreator::computeGrasp(Eigen::Affine3d contact_pose,
  Eigen::Affine3d object_pose){
    Eigen::Vector3d OC =
      object_pose.translation() - contact_pose.translation();
    Eigen::Matrix3d OC_hat;
    OC_hat << 0, -OC(2), OC(1), OC(2), 0, -OC(0), -OC(1), OC(0), 0; 
}
