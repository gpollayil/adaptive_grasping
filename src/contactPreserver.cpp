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

/* SETPERMUTATIONMATRIX */
void contactPreserver::setPermutationParams(Eigen::MatrixXd P_, int size_Q_1_){
  // Setting the permutation matrix
  P = P_;
  size_Q_1 = size_Q_1_;
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

  // If the value of the synergy is almost null, perform the relaxation
  if(std::abs(x_ref(1)) < 0.0001){
    ROS_WARN_STREAM("contactPreserver::performMinimization The synergy value is small: relaxing the constraints!.");

    // Partitioning of Q using the permutation matrix P
    if(true) std::cout << "The NON permuted Q = \n" << Q << "." << "\n-----------\n" << std::endl;
    Q = P * Q;
    if(DEBUG) std::cout << "Computed the permuted Q = \n" << Q << "." << std::endl;

    // Computing the size of Q_2
    size_Q_2 = Q.rows() - size_Q_1;
    if(DEBUG) std::cout << "Computed the size of Q_2 which is " << size_Q_2 << "." << std::endl;

    // Defining Q_1 and Q_2 and computing the null of Q_1
    Q_1 = Q.block(0, 0, size_Q_1, Q.cols());
    if(true) std::cout << "Computed Q_1: \n" << Q_1 << "\n-----------\n" << std::endl;
    Q_2 = Q.block(size_Q_1, 0, size_Q_2, Q.cols());
    if(true) std::cout << "Computed Q_2: \n" << Q_2 << "\n-----------\n" << std::endl;

    // Compute a basis of the null space by using LU decomposition
    Eigen::FullPivLU<Eigen::MatrixXd> lu_1(Q_1);
    N = lu_1.kernel();                                    // ATTENTION: overwriting the previous null matrix
    ROS_DEBUG_STREAM("N_1(Q_1) = \n" << N << ".");
    if(true) std::cout << "Computed N_1(Q_1) in contactPreserver: \n" << N << "." << std::endl;

    // Finally, compute the reference motion that preserves the contacts
    InverseBlock = (N.transpose() * (A_tilde + Q_2.transpose() * Q_2) * N).inverse();
    if(DEBUG) std::cout << "Lambda Relaxed = " << std::endl; 
    if(DEBUG) std::cout << InverseBlock*N.transpose()*A_tilde*x_d << "." << std::endl;
    x_ref = N*InverseBlock*N.transpose()*A_tilde*x_d;

    ROS_WARN_STREAM("contactPreserver::performMinimization Finished relaxing the constraints!.");
  }

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
