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

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "matrices_creator"

/**
* @brief This class is called by the adaptive_grasping method to get the
* contact jacobians, grasp matrices, contact selections and pole changes
* to compute the block matrices J, G, T and H.
*
*/

namespace adaptive_grasping {

  class matricesCreator {

  public:

    /** CONSTRUCTOR
    * @brief Default constructor for matricesCreator
    *
    * @param H_i_
    *   the basic contact selection matrix of the hand (in local frames)
    * @param world_frame_name_, palm_frame_name_
    *   the names of global and palm frames
    * @param total_joints_
    *   the total number of hand joints
    * @return null
    */
    matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
      std::string palm_frame_name_, unsigned int total_joints_);

    /** DESTRUCTOR
    * @brief Default destructor for matricesCreator
    *
    * @param null
    * @return null
    */
    ~matricesCreator();

    /** CHANGEHANDTYPE
    * @brief Function to eventually change the hand type (set new H_i)
    *
    * @param H_i_
    *   the new contact selection matrix
    * @return null
    */
    void changeHandType(Eigen::MatrixXd H_i_);

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

    /** PREPAREKDL
    * @brief Function to prepare KDL jacobian solver
    *
    * @param null
    * @return bool (success or failure)
    */
    bool prepareKDL();

  private:

    // ROS Node Handle for parsing parameters
    ros::NodeHandle nh;

    // Basic contact selection matrix
    Eigen::MatrixXd H_i;

    // Frame names for world and palm
    std::string world_frame_name;
    std::string palm_frame_name;

    // Total number of joints of the hand
    unsigned int total_joints;

    // Map of current contacts and transforms
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map;

    // Map of current joint states for fingers
    std::map<int, sensor_msgs::JointState> joints_map;

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

    /** COMPUTEGRASP
    * @brief Function to compute grasp matrix for a given contact location and
    * object pose
    *
    * @param contact_pose
    *   the pose of the contacting link on the object in global frame
    * @param object_pose
    *   the pose of the object in global frame
    * @return Eigen::MatrixXd G_i the grasp matrix for the contact
    */
    Eigen::MatrixXd computeGrasp(Eigen::Affine3d contact_pose,
      Eigen::Affine3d object_pose);

    /** COMPUTEPOLECHANGE
    * @brief Function to compute palm-contact twist pole change matrix for a
    * given contact location and palm-contact_pose
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
    * @brief Function to compute the whole block matrices J, G, T and H
    *
    * @param contacts_map_
    *   the needed details about contacts: link names, poses
    * @param joints_map_
    *   the needed details about contacting fingers: joint arrays
    * @return null - but sets the private variables J, G, T and H
    */
    void computeWholeJacobian(std::map<int, std::tuple<std::string,
      Eigen::Affine3d, Eigen::Affine3d>> contacts_map_,
      std::map<int, sensor_msgs::JointState> joints_map_);

  };

}

#endif // MATRICES_CREATOR_H
