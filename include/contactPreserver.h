#ifndef CONTACT_PRESERVER_H
#define CONTACT_PRESERVER_H

#include <iostream>
#include <Eigen/Dense>

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "contact_preserver"

#define DEBUG             1   // print out additional info

/**
* @brief This class is called by the adaptive_grasping method to compute the
* contact preserving reference motions.
*
*/

namespace adaptive_grasping {

  class contactPreserver {

  public:

    /** CONSTRUCTOR
    * @brief Default constructor for contactPreserver
    *
    * @param S_
    *   the synergy matrix of the hand (if hand fully actuated S = I)
    * @return null
    */
    contactPreserver(Eigen::MatrixXd S_);

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
    * @param S_
    *   the new synergy matrix
    * @return null
    */
    void changeHandType(Eigen::MatrixXd S_);

    /** SETGRASPSTATE
    * @brief Function to set new values for J, G, T and H
    *
    * @param J_
    *   the whole jacobian for the contacts
    * @param G_
    *   the whole grasp matrix for the contacts
    * @param T_
    *   the whole palm twist pole change matrix for the contacts
    * @param H_
    *   the new contact selection matrix
    * @return null
    */
    void setGraspState(Eigen::MatrixXd J_, Eigen::MatrixXd G_, Eigen::MatrixXd T_,
      Eigen::MatrixXd H_);

    /** SETMINIMIZATIONPARAMS
    * @brief Function to set new values for the minimization problem
    *
    * @param x_d_
    *   the desired motion of the hand given by some high level planner
    * @param A_tilde_
    *   the weight matrix in the cost function of the minimization
    * @return null
    */
    void setMinimizationParams(Eigen::VectorXd x_d_, Eigen::MatrixXd A_tilde_);

    /** PERFORMMINIMIZATION
    * @brief Function to perform the minimization using current values
    *
    * @return Eigen::VectorXd x_ref the resulting motion that preserves contacts
    */
    Eigen::VectorXd performMinimization();

  private:
    // Current contacts jacobian
    Eigen::MatrixXd J;

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

  }; // closing class

} // closing namespace



#endif // CONTACT_PRESERVER_H
