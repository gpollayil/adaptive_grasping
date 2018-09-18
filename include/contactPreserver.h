#ifndef CONTACT_PRESERVER_H
#define CONTACT_PRESERVER_H

#include <kdl/jacobian.hpp>
#include <Eigen/Core>

/**
* @brief This class is called by the adaptive_grasping method to compute the
* contact preserving reference motions.
*
*/

class contactPreserver {

public:
  /** CONSTRUCTOR
  * @brief Default constructor for contactPreserver
  *
  * @param S
  *   the synergy matrix of the hand (if hand fully actuated S = I)
  * @return null
  */
  contactPreserver(Eigen::MatrixXd S);

  /** DESTRUCTOR
  * @brief Default destructor for contactPreserver
  *
  * @param null
  * @return null
  */
  ~contactPreserver();

  /** CHANGEHANDTYPE
  * @brief Function to eventually change the hand type (set new S)
  *
  * @param S
  *   the new synergy matrix
  * @return bool (success or failure)
  */
  bool changeHandType(Eigen::MatrixXd S);

  /** SETGRASPSTATE
  * @brief Function to set new values for J, G, T and H
  *
  * @param J
  *   the whole jacobian for the contacts
  * @param G
  *   the whole grasp matrix for the contacts
  * @param T
  *   the whole palm twist pole change matrix for the contacts
  * @param H
  *   the new contact selection matrix
  * @return bool (success or failure)
  */
  bool setGraspState(KDL::Jacobian J, Eigen::MatrixXd G, Eigen::MatrixXd T,
    Eigen::MatrixXd H);

  /** SETMINIMIZATIONPARAMS
  * @brief Function to set new values for the minimization problem
  *
  * @param x_d
  *   the desired motion of the hand given by some high level planner
  * @param A_tilde
  *   the weight matrix in the cost function of the minimization
  * @return bool (success or failure)
  */
  bool setMinimizationParams(Eigen::VectorXd x_d, Eigen::MatrixXd A_tilde);

  /** PERFORM MINIMIZATION
  * @brief Function to perform the minimization using current values
  *
  * @return Eigen::VectorXd x_ref the resulting motion that preserves contacts
  */
  Eigen::VectorXd performMinimization();

private:
  // Current hand jacobian
  KDL::Jacobian J;

  // Grasp matrix
  Eigen::MatrixXd G;

  // Pole change matrix (twist of palm to contacts)
  Eigen::MatrixXd T;

  // Contact selection Matrix
  Eigen::MatrixXd H;

  // Synergy Matrix
  Eigen::MatrixXd S;

  // Planner desired motions
  Eigen::VectorXd x_d;

  // Minimization weights
  Eigen::MatrixXd A_tilde;

  // Contact relation matrix
  Eigen::MatrixXd Q;

  // Null space basis of Q
  Eigen::MatrixXd N;
}

#endif // CONTACT_PRESERVER_H
