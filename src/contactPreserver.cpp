#include "contactPreserver.h"

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
contactPreserver::contactPreserver(Eigen::MatrixXd S_){
  // Set the synergy matrix
  changeHandType(S_);
}

/* DESTRUCTOR */
contactPreserver::~contactPreserver(){
  // Nothing to do
}

/* CHANGEHANDTYPE */
bool contactPreserver::changeHandType(Eigen::MatrixXd S_){
  // Set the new synergy matrix
  S = S_;
}

/* SETGRASPSTATE */
bool contactPreserver::setGraspState(KDL::Jacobian J_, Eigen::MatrixXd G_, Eigen::MatrixXd T_,
  Eigen::MatrixXd H_){
    // Set the new J, G, T and H matrices
    J = J_; G = G_; T = T_; H = H_;
  }

/* SETMINIMIZATIONPARAMS */
bool contactPreserver::setMinimizationParams(Eigen::VectorXd x_d_, Eigen::MatrixXd A_tilde_){
  // Set the new desired motion vector and weight matrix
  x_d = x_d_; A_tilde = A_tilde_;
}

/* PERFORMMINIMIZATION */
Eigen::VectorXd contactPreserver::performMinimization(){
  // Resize Q to be of correct size
  Q.resize(H.rows(), x_d.norm());

  // Now create the block matrix
  Q << H*J.data*S, H*T, Eigen::MatrixXd::Zero(H.rows(), G.rows())-H*G.transpose();

  // Compute a basis of the null space by using LU decomposition
  Eigen::FullPivLU<Eigen::MatrixXd> lu(Q);
  N = lu.kernel();

  // Finally, compute the reference motion that preserves the contacts
  Eigen::VectorXd x_ref = N*(N.transpose()*A_tilde*N).inverse()*N.transpose()*A_tilde*x_d;
}
