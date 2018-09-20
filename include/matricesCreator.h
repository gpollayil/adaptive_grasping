#ifndef MATICES_CREATOR_H
#define MATICES_CREATOR_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

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
    * @return null
    */
    matricesCreator(Eigen::MatrixXd H_i_);

    /** DESTRUCTOR
    * @brief Default destructor for matricesCreator
    *
    * @param null
    * @return null
    */
    ~matricesCreator();

    /** SETCONTACTSMAP
    * @brief Function to set the map that has details about contacts
    *
    * @param contact_fingers_map_
    *   the map containing int ids and transforms of contactinf fingers
    * @return bool (success or failure)
    */
    bool setContactsMap(std::map<int, Eigen::Affine3d> contact_fingers_map_);

  private:

    // Basic contact selection matrix
    Eigen::MatrixXd H_i;

    // Map of current contacts and transforms
    std::map<int, Eigen::Affine3d> contact_fingers_map;

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
    KDL::ChainJntToJacSolver jacobian_solver;

  }

}
