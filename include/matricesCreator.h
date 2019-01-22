#ifndef MATRICES_CREATOR_H
#define MATRICES_CREATOR_H

#include <map>
#include <tuple>
#include <boost/scoped_ptr.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

/**
* @brief This class is called by the adaptive_grasping method to get the
* contact jacobians, grasp matrices, contact selections and pole changes
* to compute the block matrices J, G, T and H.
*
*/

namespace adaptive_grasping {

  class matricesCreator {

  public:

    /** DEFAULT CONSTRUCTOR
    * @brief Default constructor for matricesCreator
    *
    * @param null
    * @return null
    */
    matricesCreator();

    /** CONSTRUCTOR
    * @brief Overloaded constructor for matricesCreator
    *
    * @param H_i_
    *   the basic contact selection matrix of the hand (in local frames)
    * @param world_frame_name_, palm_frame_name_
    *   the names of global and palm frames
    * @param joint_numbers_
    *   the number of joints of each finger in a vector
    * @return null
    */
    matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
      std::string palm_frame_name_, std::vector<int> joint_numbers_);

    /** DESTRUCTOR
    * @brief Default destructor for matricesCreator
    *
    * @param null
    * @return null
    */
    ~matricesCreator();

    // A boolean for checking if the object has been initialized
    bool initialized = false;

    /** INITIALIZE
    * @brief A public function for initializing the object
    *
    * @param H_i_
    *   the basic contact selection matrix of the hand (in local frames)
    * @param world_frame_name_, palm_frame_name_
    *   the names of global and palm frames
    * @param joint_numbers_
    *   the number of joints of each finger in a vector
    * @return null
    */
    bool initialize(Eigen::MatrixXd H_i_, std::string world_frame_name_,
      std::string palm_frame_name_, std::vector<int> joint_numbers_);

    /** CHANGECONTACTTYPE
    * @brief Function to eventually change the hand type (set new H_i)
    *
    * @param H_i_
    *   the new contact selection matrix
    * @return null
    */
    void changeContactType(Eigen::MatrixXd H_i_);

    /** CHANGEFRAMENAMES
    * @brief Function to eventually change the world and palm frame names
    *
    * @param world_frame_name_, palm_frame_name_
    *   the new frame names
    * @return null
    */
    void changeFrameNames(std::string world_frame_name_,
      std::string palm_frame_name_);

    /** SETCONTACTSMAP
    * @brief Function to set the map that has details about contacts
    *
    * @param contacts_map_
    *   the map containing int ids and transforms of contacting fingers
    * @return null
    */
    void setContactsMap(std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map_);

    /** SETJOINTSMAP
    * @brief Function to set the map that has details about finger jointstates
    *
    * @param joints_map_
    *   the map containing int ids and jointstates of contacting fingers
    * @return null
    */
    void setJointsMap(std::map<int, sensor_msgs::JointState> joints_map_);

    /** SETOBJECTPOSE
    * @brief Function to set the current object pose
    *
    * @param object_pose_
    *   the object pose
    * @return null
    */
    void setObjectPose(Eigen::Affine3d object_pose_);

    /** SETPERMUTATIONVECTOR
    * @brief Function to set the permutation vector for later perm. matrix computation
    *
    * @param p_vector_
    *   the permutation vector
    * @return null
    */
    void setPermutationVector(Eigen::VectorXd p_vector_);

    /** PREPAREKDL
    * @brief Function to prepare KDL jacobian solver
    *
    * @param null
    * @return bool (success or failure)
    */
    bool prepareKDL();

    /** COMPUTEALLMATRICES
    * @brief Function to compute all of the matrices J, G, T and H at once
    *
    * @param null
    * @return null - but sets the private variables J, G, T and H
    */
    void computeAllMatrices();

    /** READALLMATRICES
    * @brief Function to read all of the matrices J, G, T, P and H at once
    *
    * @param read_J
    *   the reference to which the whole jacobian should be saved
    * @param read_G
    *   the reference to which the whole grasp should be saved
    * @param read_T
    *   the reference to which the whole pole change should be saved
    * @param read_H
    *   the reference to which the contact selection should be saved
    * @param read_P
    *   the permutation matrix
    * @return null
    */
    void readAllMatrices(Eigen::MatrixXd& read_J, Eigen::MatrixXd& read_G,
      Eigen::MatrixXd& read_T, Eigen::MatrixXd& read_H, Eigen::MatrixXd& read_P);

  private:

    // ROS Node Handle for parsing parameters
    ros::NodeHandle nh;

    // Basic contact selection matrix
    Eigen::MatrixXd H_i;

    // Permutation vector and matrix for relaxed minimization and size of whole Q_1
    Eigen::VectorXd p_vector;
    Eigen::MatrixXd P;

    // Frame names for world and palm
    std::string world_frame_name;
    std::string palm_frame_name;

    // Number of joints of each finger
    std::vector<int> joint_numbers;

    // Total number of joints of the hand
    unsigned int total_joints;

    // Map of current contacts and transforms (contact w.r.t. world and palm)
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map;

    // Map of current joint states for fingers
    std::map<int, sensor_msgs::JointState> joints_map;

    // Palm to World transform (set by compute pole change)
    Eigen::Affine3d Palm_to_World;

    // Contacts jacobian
    Eigen::MatrixXd J;

    // Grasp matrix
    Eigen::MatrixXd G;

    // Pole change matrix (twist of palm to contacts)
    Eigen::MatrixXd T;

    // Contacts selection Matrix
    Eigen::MatrixXd H;

    // Hand kinematic tree
    KDL::Tree robot_kin_tree;

    // Finger kinematic chain
    KDL::Chain finger_kin_chain;

    // Finger joint array
    KDL::JntArray finger_joint_array;

    // Jacobian solver from joint array
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
    // KDL::ChainJntToJacSolver jacobian_solver;

    // Object pose
    Eigen::Affine3d object_pose;

    /** COMPUTEJACOBIAN
    * @brief Function to compute jacobian for a given chain and joint array
    *
    * @param chain
    *   the kinematic chain
    * @param q_
    *   the state of the joints of the chain
    * @return KDL::Jacobian J_i jacobian of chain in q_
    */
    KDL::Jacobian computeJacobian(KDL::Chain chain, KDL::JntArray q_);

    /** TRANSFORMJACOBIAN
    * @brief Function to express jacobian into world frames from palm frame
    *
    * @param Jac
    *   the jacobian in palm frame
    * @return KDL::Eigen::MatrixXd J_w the jacobian in world frame
    */
    Eigen::MatrixXd transformJacobian(KDL::Jacobian Jac);

    /** COMPUTEGRASP
    * @brief Function to compute grasp matrix for a given contact location and
    * object pose
    *
    * @param contact_pose
    *   the pose of the contacting link on the object in global frame
    * @param object_pose_
    *   the pose of the object in global frame
    * @return Eigen::MatrixXd G_i the grasp matrix for the contact
    */
    Eigen::MatrixXd computeGrasp(Eigen::Affine3d contact_pose,
      Eigen::Affine3d object_pose_);

    /** COMPUTEPOLECHANGE
    * @brief Function to compute palm-contact twist pole change matrix for a
    * given contact location and palm-contact pose
    *
    * @param contact_pose
    *   the pose of the contacting link on the object in global frame
    * @param pc_pose
    *   the transformation fron palm to contact in palm frame
    * @return Eigen::MatrixXd T_i the palm-contact twist pole change matrix
    */
    Eigen::MatrixXd computePoleChange(Eigen::Affine3d contact_pose,
      Eigen::Affine3d pc_pose);

    /** GETFINGERJOINTS
    * @brief Function to get KDL::JntArray from joints map and finger id
    *
    * @param joints_map_
    *   the needed details about contacting fingers: joint arrays
    * @param finger_id_
    *   the id of the finger
    * @return KDL::JntArray q the joint array of the finger specified by id
    */
    KDL::JntArray getFingerJoints(std::map<int, sensor_msgs::JointState> joints_map_,
      int finger_id_);

    /** COMPUTEWHOLEJACOBIAN
    * @brief Function to compute the whole block matricx J
    *
    * @param contacts_map_
    *   the needed details about contacts: link names, poses
    * @param joints_map_
    *   the needed details about contacting fingers: joint arrays
    * @return null - but sets the private variable J
    */
    void computeWholeJacobian(std::map<int, std::tuple<std::string,
      Eigen::Affine3d, Eigen::Affine3d>> contacts_map_,
      std::map<int, sensor_msgs::JointState> joints_map_);

    /** COMPUTEWHOLEGRASP
    * @brief Function to compute the whole block matrix G
    *
    * @param contacts_map_
    *   the needed details about contacts: link names, poses
    * @return null - but sets the private variable G
    */
    void computeWholeGrasp(std::map<int, std::tuple<std::string,
      Eigen::Affine3d, Eigen::Affine3d>> contacts_map_);

    /** COMPUTEWHOLEPOLECHANGE
    * @brief Function to compute the whole block matrix T
    *
    * @param contacts_map_
    *   the needed details about contacts: link names, poses
    * @return null - but sets the private variable T
    */
    void computeWholePoleChange(std::map<int, std::tuple<std::string,
      Eigen::Affine3d, Eigen::Affine3d>> contacts_map_);

    /** COMPUTEWHOLECONTACTSELECTION
    * @brief Function to compute the whole block matrix H
    *
    * @param contacts_map_
    *   the needed details about contacts: link names, poses
    * @return null - but sets the private variable H
    */
    void computeWholeContactSelection(std::map<int, std::tuple<std::string,
      Eigen::Affine3d, Eigen::Affine3d>> contacts_map_);

    /** COMPUTEPERMUTATIONMATRIX
    * @brief Function to compute the permutation matrix P
    *
    * @param p_vector_
    *   the permutation vector that shows the desired order of rows
    * @param contacts_num_
    *   the number of finger contacts
    * @return null - but sets the private variable P
    */
    void computePermutationMatrix(Eigen::VectorXd p_vector_, int contacts_num_);

  };

}

#endif // MATRICES_CREATOR_H
