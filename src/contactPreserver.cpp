#include "contactPreserver.h"
#include "ros/ros.h"
#include "utils/pseudo_inversion.h"

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

  // Setting temporary values of x_d and x_d_old
  x_d_old = Eigen::VectorXd::Ones(S_.cols() + 6 + 6);

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
  } else if(relaxation_order < x_d.size()) {
    R = Eigen::MatrixXd::Identity(relaxation_order, Q_tilde.rows());
  } else {
    int residual = relaxation_order - x_d.size();
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

    std::cout << "contactPreserver::setRMatrix Created R =" << std::endl; std::cout << R << std::endl;
  }

  // Compute R_bar as N(R) transpose
  Eigen::FullPivLU<Eigen::MatrixXd> lu(R);
  R_bar = lu.kernel().transpose();

  std::cout << "contactPreserver::setRMatrix Created R_bar =" << std::endl; std::cout << R_bar << std::endl;

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
    A_tilde.block(A_tilde_parsed.rows(), A_tilde_parsed.rows(), rem_rows, rem_rows) = Eigen::MatrixXd::Identity(rem_rows, rem_rows);
  }

  std::cout << "contactPreserver::updateAMatrix Updated A_tilde =" << std::endl; std::cout << A_tilde << std::endl;
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
  }

  x_d_old = x_d;

  // Now if necessary changing the matrices R and R_bar or eventually reset them
  setRMatrix();

  // Check the first condition of algorithm
  pseudo_inverse(R_bar * Q_tilde, pinv_R_bar_Q_tilde, false);             // Undamped pseudo inversion of (R_bar * Q_tilde)
  if((R_bar * Q_tilde * pinv_R_bar_Q_tilde * R_bar * y - R_bar * y).isMuchSmallerThan(0.0001)){
    // Compute a basis of the null space of Q_tilde by using LU decomposition
    Eigen::FullPivLU<Eigen::MatrixXd> lu(Q_tilde);
    N_tilde = lu.kernel();
    ROS_DEBUG_STREAM("N_tilde(Q) = \n" << N_tilde << ".");

    // Print message for debug
    if(DEBUG) std::cout << "Computed N_tilde(Q) in contactPreserver!" << std::endl;

    // Updating A_tilde to comply with the dimensions of R
    updateAMatrix();

    // Computing the solution (formulas in paper)
    C = N_tilde.transpose() * Q_tilde.transpose() * R.transpose() * A_tilde * R;
    x_star = pinv_R_bar_Q_tilde * R_bar * y;
    x_ref = x_star + N_tilde * (C * Q_tilde * N_tilde).inverse() * C * (y - Q_tilde * x_star);

    // Checking the second condition of algorithm
    if(x_ref.head(S.cols()).norm() < 0.0001){
      /*  If the condition for norm is not valid, relax (increase relaxation_order) 
        Recomputation of the R matrices will be performed by setRMatrix at next iteration 
      */
      relaxation_order += 1;
      x_ref = Eigen::VectorXd::Zero(x_d.size());            // Null vector is returned to keep the robot still until good solution is found
    }
  } else {
    /*  If the condition for solution is not valid, relax (increase relaxation_order) 
        Recomputation of the R matrices will be performed by setRMatrix at next iteration 
    */
    relaxation_order += 1;
    x_ref = Eigen::VectorXd::Zero(x_d.size());            // Null vector is returned to keep the robot still until good solution is found
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
  std::cout << "N_tilde =" << std::endl; std::cout << N_tilde << std::endl;
}
