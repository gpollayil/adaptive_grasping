#include "contactPreserver.h"
#include "ros/ros.h"
#include "utils/pseudo_inversion.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "contact_preserver"
#define DEBUG             0   // print out additional info
#define N_DEBUG           0   // sends as reference column of N(Q)
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

  // Setting temporary values of x_d and x_d_old
  x_d_old = Eigen::VectorXd::Ones(S_.cols() + 6 + 6);
  x_ref_old = Eigen::VectorXd::Zero(x_d_old.size());
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
  x_d = x_d_; A_tilde_parsed = A_tilde_;
}

/* SETPERMUTATIONMATRIX */
void contactPreserver::setPermutationParams(Eigen::MatrixXd P_, int num_contacts_){
  // Setting the permutation matrix
  P = P_;
  num_contacts = num_contacts_;
}

/* SETRMATRIX */
bool contactPreserver::setRMatrix(){
  // Cannot relax more than the rows of Q_tilde
  if(relaxation_order > Q_tilde.rows()) return false;

  // Build R case by case
  if(relaxation_order == 0){
    R = Eigen::MatrixXd::Zero(1, Q_tilde.rows());
  } else if(relaxation_order <= x_d.size()) {
    R = Eigen::MatrixXd::Identity(relaxation_order, Q_tilde.rows());
  } else {
    int residual = relaxation_order - x_d.size();

    if(DEBUG) std::cout << "setRMatrix residual = " << residual << " - num_contacts = " << num_contacts << " - x_d.size() = " <<
      x_d.size() << " - P.cols() = " << P.cols() << std::endl;

    R = Eigen::MatrixXd::Zero(x_d.size() + num_contacts * residual, x_d.size() + P.cols());
    R.block(0, 0, x_d.size(), x_d.size()) = Eigen::MatrixXd::Identity(x_d.size(), x_d.size());

    int index_x = x_d.size();
    int index_y = x_d.size();
    if(DEBUG) ROS_INFO_STREAM("contactPreserver::setRMatrix Entering the blocks creating for.");
    for(int i = 0; i < num_contacts; i++){
      R.block(index_x, index_y, residual, P.cols() / num_contacts) = P.block(0, 0, residual, P.cols() / num_contacts);
      index_x += residual;
      index_y += P.cols() / num_contacts;
    }
  }

  if(DEBUG) std::cout << "contactPreserver::setRMatrix Created R =" << std::endl; 
  if(DEBUG) std::cout << R << std::endl;

  // Compute R_bar as N(R) transpose
  if(relaxation_order == 0){
    R_bar = Eigen::MatrixXd::Identity(Q_tilde.rows(), Q_tilde.rows());
  } else {
    Eigen::FullPivLU<Eigen::MatrixXd> lu(R);
    R_bar = lu.kernel().transpose();
  }

  if(DEBUG) std::cout << "contactPreserver::setRMatrix Created R_bar =" << std::endl; 
  if(DEBUG) std::cout << R_bar << std::endl;

  return true;
}

/* UPDATEAMATRIX */
void contactPreserver::updateAMatrix(){
  // If R matrix is smaller than A_tilde_parsed no need to add extra diagonal identity
  if(R.rows() <= A_tilde_parsed.rows()){
    A_tilde = A_tilde_parsed.block(0, 0, R.rows(), R.rows());
  } else {
    A_tilde = Eigen::MatrixXd::Zero(R.rows(), R.rows());
    A_tilde.block(0, 0, A_tilde_parsed.rows(), A_tilde_parsed.rows()) = A_tilde_parsed;
    int rem_rows = R.rows() - A_tilde_parsed.rows();
    // For now the weight on the contacts is hard-coded (TODO: parse also this param)
    A_tilde.block(A_tilde_parsed.rows(), A_tilde_parsed.rows(), rem_rows, rem_rows) = 10000 * Eigen::MatrixXd::Identity(rem_rows, rem_rows);
  }

  if(DEBUG) std::cout << "contactPreserver::updateAMatrix Updated A_tilde =" << std::endl; 
  if(DEBUG) std::cout << A_tilde << std::endl;
}

/* PERFORMMINIMIZATION */
bool contactPreserver::performMinimization(Eigen::VectorXd& x_result){
  // Resize Q to be of correct size
  Q.resize(H.rows(), x_d.size());

  // Print message for debug
  if(DEBUG) std::cout << "Resized Q in contactPreserver!" << std::endl;

  // Now create the block matrix
  Eigen::MatrixXd NullMatrix = Eigen::MatrixXd::Zero(H.rows(), G.rows());
  Q << H*J*S, H*T, NullMatrix-H*G.transpose();

  // For debugging purpouses (real line is above)
  if(N_DEBUG){
    Eigen::FullPivLU<Eigen::MatrixXd> luN_debug(Q);
    Eigen::MatrixXd N_debug = luN_debug.kernel();
    x_result = N_debug.col(0);
    return true;
  }
  
  // Comparing Q with the old one in order to reset relaxation
  if(first_it){
    Q_old = Q;
  } else {
    if(!(Q - Q_old).isMuchSmallerThan(0.0001) && !x_ref.isMuchSmallerThan(0.0001) && (relaxation_order > x_d.size())){
      relaxation_order = 0;                     // Reset relaxation
      if(DEBUG) ROS_WARN_STREAM("Resetting relaxation because Q changed.");
      if(DEBUG) std::cout << "----------------" << std::endl;
      if(DEBUG) std::cout << "Q = " << Q << std::endl;
      if(DEBUG) std::cout << "Q_old = " << Q_old << std::endl;
      if(DEBUG) std::cout << "----------------" << std::endl;
    }
    Q_old = Q;
  }

  // Print message for debug
  if(DEBUG) std::cout << "Computed Q in contactPreserver!" << std::endl;

  // Now create Q_tilde by appending Q under Identity matrix
  Q_tilde.resize(x_d.size() + H.rows(), x_d.size());
  Q_tilde << Eigen::MatrixXd::Identity(x_d.size(), x_d.size()), Q;

  // Print message for debug
  if(DEBUG) std::cout << "Computed Q_tilde in contactPreserver!" << std::endl;

  // Compute vector y
  y.resize(x_d.size() + H.rows());
  y << x_d, Eigen::VectorXd::Zero(H.rows());

  // If the desired motion has changed, reset R and R_bar
  if(!(x_d - x_d_old).isMuchSmallerThan(0.0001) || first_it){
    relaxation_order = 0;               // Nothing is to be relaxed
    if(first_it) first_it = false;
    if(DEBUG) ROS_WARN_STREAM("Resetting relaxation because x_d changed.");
  }

  x_d_old = x_d;

  if(DEBUG) std::cout << "x_d in CP = " << x_d << std::endl;
  if(DEBUG) std::cout << "x_d_old in CP = " << x_d_old << std::endl;

  // Now if necessary changing the matrices R and R_bar or eventually reset them
  if(!setRMatrix()){
    ROS_ERROR_STREAM("The Relaxation iterations finished without leading to a solution!!! Waiting for another x_d!");
  }

  if(DEBUG) std::cout << "-- Step 1 --" << std::endl;

  // Computing pseudoinverse for next condition
  pseudo_inverse(R_bar * Q_tilde, pinv_R_bar_Q_tilde, false);             // Undamped pseudo inversion of (R_bar * Q_tilde)
  if(pinv_R_bar_Q_tilde.hasNaN()){
    pseudo_inverse(R_bar * Q_tilde, pinv_R_bar_Q_tilde, true);            // Damped pseudo inversion of (R_bar * Q_tilde)
  }

  if(DEBUG) std::cout << "-- Step 2 --" << std::endl;

  if(DEBUG) std::cout << "----------------" << std::endl;
  if(DEBUG) std::cout << "pinv_R_bar_Q_tilde = " << pinv_R_bar_Q_tilde << std::endl;
  if(DEBUG) std::cout << "----------------" << std::endl;

  if(DEBUG){
    std::cout << "----------------" << std::endl;
    std::cout << "relaxation_order = " << relaxation_order << std::endl;
    std::cout << "----------------" << std::endl;
  }

  // Check if all constraints have been relaxed
  bool all_relaxed = relaxation_order >= Q_tilde.rows();

  // Check the first condition of algorithm
  if((R_bar * Q_tilde * pinv_R_bar_Q_tilde * R_bar * y - R_bar * y).isMuchSmallerThan(0.0001) && !all_relaxed){
    // Compute a basis of the null space of Q_tilde by using LU decomposition
    if(DEBUG) std::cout << "-- Step 3 --" << std::endl;
    if(DEBUG) std::cout << "R_bar = " << R_bar << std::endl;
    if(DEBUG) std::cout << "Q_tilde = " << Q_tilde << std::endl;
    if(DEBUG) std::cout << "R_bar * Q_tilde = " << R_bar * Q_tilde << std::endl;

    Eigen::FullPivLU<Eigen::MatrixXd> luN(R_bar * Q_tilde);
    N_tilde = luN.kernel();
    ROS_DEBUG_STREAM("N_tilde(Q) = \n" << N_tilde << ".");

    if(DEBUG) std::cout << "-- Step 4 --" << std::endl;

    if(DEBUG) std::cout << "----------------" << std::endl;
    if(DEBUG) std::cout << "N_tilde = " << N_tilde << std::endl;
    if(DEBUG) std::cout << "----------------" << std::endl;

    // Print message for debug
    if(DEBUG) std::cout << "Computed N_tilde(Q) in contactPreserver!" << std::endl;

    // Updating A_tilde to comply with the dimensions of R
    updateAMatrix();

    // Computing the solution (formulas in paper)
    C = N_tilde.transpose() * Q_tilde.transpose() * R.transpose() * A_tilde * R;
    x_star = pinv_R_bar_Q_tilde * R_bar * y;

    // Check invertibility of block expression
    Eigen::FullPivLU<Eigen::MatrixXd> lu(C * Q_tilde * N_tilde);
    if(!lu.isInvertible()){
      if(DEBUG) ROS_FATAL_STREAM("Non invertible C * Q_tilde * N_tilde!");
      if(relaxation_order <= Q_tilde.rows()) relaxation_order += 1;
      x_ref = x_ref_old;            // Old vector is returned until good solution is found
      x_result = x_ref;
      if(DEBUG || true) ROS_WARN_STREAM("Relaxing because Non invertible C * Q_tilde * N_tilde.");
      return false;
    }

    // Compute reference
    x_ref = x_star + N_tilde * (C * Q_tilde * N_tilde).inverse() * C * (y - Q_tilde * x_star);
    
    // **********************************************************************************************//
    // JacobiSVD<MatrixXd> svd(C * Q_tilde * N_tilde,  ComputeThinV | ComputeThinU);
    // Eigen::MatrixXd temp_matriv = svd.singularValues().asDiagonal();
    // for(int i = 0; i < temp_matriv.rows(); i++)
    // {
    //   if (std::abs(temp_matriv(i,i)) > 0.001) temp_matriv(i,i) = 1.0 / temp_matriv(i,i);
    // }
    // Eigen::MatrixXd some_inv = (svd.matrixV().transpose() * temp_matriv * svd.matrixU().transpose());
    // x_ref = x_star + N_tilde * some_inv * C * (y - Q_tilde * x_star);
    // **********************************************************************************************//

    if(DEBUG) std::cout << "----------------" << std::endl;
    if(DEBUG) std::cout << "x_star = " << x_star << std::endl;
    if(DEBUG) std::cout << "N_tilde = " << N_tilde << std::endl;
    if(DEBUG) std::cout << "(C * Q_tilde * N_tilde) = " << (C * Q_tilde * N_tilde) << std::endl;
    if(DEBUG) std::cout << "(C * Q_tilde * N_tilde).inverse() = " << (C * Q_tilde * N_tilde).inverse() << std::endl;
    if(DEBUG) std::cout << "C = " << C << std::endl;
    if(DEBUG) std::cout << "(y - Q_tilde * x_star) = " << (y - Q_tilde * x_star) << std::endl;
    if(DEBUG) std::cout << "----------------" << std::endl;
    
    // Checking the second condition of algorithm
    if(x_ref.head(S.cols()).norm() < 0.000001){
      /*  If the condition for norm is not valid, relax (increase relaxation_order) 
        Recomputation of the R matrices will be performed by setRMatrix at next iteration 
      */
      if(relaxation_order <= Q_tilde.rows()) relaxation_order += 1;
      x_ref = x_ref_old;            // Old vector is returned until good solution is found
      x_result = x_ref;
      if(DEBUG || true) ROS_WARN_STREAM("Relaxing because small joint velocity reference.");
      return false;
    }
  } else {
    /*  If the condition for solution is not valid, relax (increase relaxation_order) 
        Recomputation of the R matrices will be performed by setRMatrix at next iteration 
    */
    if(relaxation_order <= Q_tilde.rows()) relaxation_order += 1;
    x_ref = x_ref_old;            // Old vector is returned until good solution is found
    x_result = x_ref;
    if(DEBUG || true) ROS_WARN_STREAM("Relaxing because no particular solution found.");
    return false;
  }

  // DEBUG PRINTS
  if(DEBUG) std::cout << "----------------" << std::endl;
  if(DEBUG) std::cout << "Q_tilde = " << Q_tilde << std::endl;
  if(DEBUG) std::cout << "y = " << y << std::endl;
  if(DEBUG) std::cout << "R_bar * y = " << R_bar * y << std::endl;
  if(DEBUG) std::cout << "----------------" << std::endl;

  // Saving to old and return contact preserving solution
  x_ref_old = x_ref;
  x_result = x_ref;
  ROS_ERROR_STREAM("A new reference has been sent! Yahoo!.");
  return true;
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
  std::cout << "N_tilde =" << std::endl; std::cout << N_tilde << std::endl;
}
