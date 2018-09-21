#ifndef MATRICES_CREATOR_H
#define MATRICES_CREATOR_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>

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
    * @return null
    */
    matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
      std::string palm_frame_name_);

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
    *   the map containing int ids and transforms of contactinf fingers
    * @return null
    */
    void setContactsMap(std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map_);

    /** SETOBJECTPOSE
    * @brief Function to set the current object pose
    *
    * @param object_pose_
    *   the object pose
    * @return null
    */
    void setObjectPose(Eigen::Affine3d object_pose_);

  private:

    // Basic contact selection matrix
    Eigen::MatrixXd H_i;

    // Frame names for world and palm
    std::string world_frame_name;
    std::string palm_frame_name;

    // Map of current contacts and transforms
    std::map<int, std::tuple<std::string, Eigen::Affine3d,
      Eigen::Affine3d>> contacts_map;

    // Contacts jacobian
    KDL::Jacobian J;

    // Grasp matrix
    Eigen::MatrixXd G;

    // Pole change matrix (twist of palm to contacts)
    Eigen::MatrixXd T;

    // Contacts selection Matrix
    Eigen::MatrixXd H;

    // Hand kinematic tree
    KDL::Tree hand_kin_tree;

    // Finger kinematic chain
    KDL::Chain finger_kin_chain;

    // Finger joint array
    KDL::JntArray finger_joint_array;

    // Jacobian solver from joint array
    // KDL::ChainJntToJacSolver jacobian_solver;

    // Object pose
    Eigen::Affine3d object_pose;

  };

}

#endif // MATRICES_CREATOR_H