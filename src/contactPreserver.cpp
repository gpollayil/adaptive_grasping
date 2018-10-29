#include "contactPreserver.h"
#include "ros/ros.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "contact_preserver"
#define DEBUG             0   // print out additional info

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* DEFAULT CONSTRUCTOR */
contactPreserver::contactPreserver(){
  // Nothing to do here
}

/* CONSTRUCTOR */
contactPreserver::contactPreserver(Eigen::MatrixXd S_){
  // Initializing the object
  initialized = initialize(S_);
}

/* DESTRUCTOR */
contactPreserver::~contactPreserver(){
  // Nothing to do
}

/* INITIALIZE */
bool contactPreserver::initialize(Eigen::MatrixXd S_){
  // Set the synergy matrix
  changeHandType(S_);
}

/* CHANGEHANDTYPE */
void contactPreserver::changeHandType(Eigen::MatrixXd S_){
  // Set the new synergy matrix
  S = S_;
  ROS_DEBUG_STREAM("Changed the Synergy Matrix inside contact preserver!!!");
}

/* SETGRASPSTATE */
void contactPreserver::setGraspState(Eigen::MatrixXd J_, Eigen::MatrixXd G_,
  Eigen::MatrixXd T_, Eigen::MatrixXd H_){
    // Set the new J, G, T and H matrices
    J = J_; G = G_; T = T_; H = H_;
}

/* SETMINIMIZATIONPARAMS */
void contactPreserver::setMinimizationParams(Eigen::VectorXd x_d_,
  Eigen::MatrixXd A_tilde_){
  // Set the new desired motion vector and weight matrix
  x_d = x_d_; A_tilde = A_tilde_;
}

/* PERFORMMINIMIZATION */
Eigen::VectorXd contactPreserver::performMinimization(){
  // Resize Q to be of correct size
  Q.resize(H.rows(), x_d.size());

  // Print message for debug
  if(DEBUG) std::cout << "Resized Q in contactPreserver!" << std::endl;

  // Now create the block matrix
  Eigen::MatrixXd NullMatrix = Eigen::MatrixXd::Zero(H.rows(), G.rows());
  Q << H*J*S, H*T, NullMatrix-H*G.transpose();

  // Print message for debug
  if(DEBUG) std::cout << "Computed Q in contactPreserver!" << std::endl;

  // Compute a basis of the null space by using LU decomposition
  Eigen::FullPivLU<Eigen::MatrixXd> lu(Q);
  N = lu.kernel();
  ROS_DEBUG_STREAM("N(Q) = \n" << N << ".");

  // Print message for debug
  if(DEBUG) std::cout << "Computed N(Q) in contactPreserver!" << std::endl;

  // Finally, compute the reference motion that preserves the contacts
  Eigen::MatrixXd InverseBlock = (N.transpose()*A_tilde*N).inverse();
  if(DEBUG) std::cout << "Lambda = " << std::endl; 
  if(DEBUG) std::cout << InverseBlock*N.transpose()*A_tilde*x_d << "." << std::endl;
  Eigen::VectorXd x_ref = N*InverseBlock*N.transpose()*A_tilde*x_d;

  // Return contact preserving solution
  return x_ref;
  
  // For debugging purpouses (real line is above)
  // Eigen::VectorXd null_vec_debug = N.col(0);
  // return null_vec_debug;
}

/* PRINTALL */
void contactPreserver::printAll(){
  // Print to screen the main private variables
  std::cout << "J =" << std::endl; std::cout << J << std::endl;
  std::cout << "G =" << std::endl; std::cout << G << std::endl;
  std::cout << "T =" << std::endl; std::cout << T << std::endl;
  std::cout << "H =" << std::endl; std::cout << H << std::endl;
  std::cout << "S =" << std::endl; std::cout << S << std::endl;
  std::cout << "A_tilde =" << std::endl; std::cout << A_tilde << std::endl;
  std::cout << "x_d =" << std::endl; std::cout << x_d << std::endl;
  std::cout << "Q =" << std::endl; std::cout << Q << std::endl;
  std::cout << "N =" << std::endl; std::cout << N << std::endl;
}
