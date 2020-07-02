#include "contactPreserver.h"
#include "ros/ros.h"
#include "utils/pseudo_inversion.h"

#define DEBUG               0   // print out additional info
#define N_DEBUG             0   // sends as reference column of N(Q)
#define USE_RP              0   // task inversion is performed using RP

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* DEFAULT CONSTRUCTOR */
contactPreserver::contactPreserver() {
	// Nothing to do here
}

/* CONSTRUCTOR */
contactPreserver::contactPreserver(Eigen::MatrixXd S_) {
	// Initializing the object
	initialized = initialize(S_);
}

/* OTHER OVERLOADED CONSTRUCTOR */
contactPreserver::contactPreserver(Eigen::MatrixXd S_, int num_tasks_, std::vector<int> dim_tasks_,
                                   std::vector<int> prio_tasks_, double lambda_max_, double epsilon_) {
	// Initializing the object
	initialized = initialize(S_);

	// Initializing RP tasks stuff
	this->initialize_tasks(num_tasks_, dim_tasks_, prio_tasks_, lambda_max_, epsilon_);
}

/* DESTRUCTOR */
contactPreserver::~contactPreserver() {
	// Nothing to do
}

/* INITIALIZE */
bool contactPreserver::initialize(Eigen::MatrixXd S_) {
	// Set the synergy matrix
	changeHandType(S_);

	// Initialize xi_o
	this->xi_o = Eigen::VectorXd::Zero(6);

	// Setting temporary values of x_d and x_d_old
	x_d_old = Eigen::VectorXd::Ones(1 + 6);
	ROS_WARN_STREAM("The number of rows of x_d_old is " << this->x_d_old.rows());
	x_ref_old = Eigen::VectorXd::Zero(x_d_old.size());
}

/* INITIALIZETOPICS */
bool contactPreserver::initialize_topics(std::string object_twist_topic_name_, ros::NodeHandle nh) {
	// Setting the topic
	this->object_twist_topic_name = object_twist_topic_name_;
	this->cp_nh_ptr = std::unique_ptr<ros::NodeHandle>(&nh);
	geometry_msgs::Twist::ConstPtr tmp_twist_in = ros::topic::waitForMessage<geometry_msgs::Twist>(
			this->object_twist_topic_name, nh, ros::Duration(2.0));
	this->obj_twist_sub = this->cp_nh_ptr->subscribe(this->object_twist_topic_name, 10,
	                                                 &contactPreserver::object_twist_callback, this);
}

/* INITIALIZE */
bool contactPreserver::initialize_tasks(int num_tasks_, std::vector<int> dim_tasks_, std::vector<int> prio_tasks_,
                                        double lambda_max_, double epsilon_) {
	// Set the tasks stuff for RP Manager
	this->num_tasks = num_tasks_;
	this->dim_tasks = dim_tasks_;
	this->prio_tasks = prio_tasks_;

	// Double checking for consistency of previous vectors
	if (this->dim_tasks.size() <= this->num_tasks || this->prio_tasks.size() <= this->num_tasks ||
	    this->dim_tasks.size() != this->prio_tasks.size()) {
		ROS_ERROR(
				"Attention!!! There is some incongurency between num_tasks, dim_tasks and prio_tasks... This won't work anymore!");
	}
	this->lambda_max = lambda_max_;
	this->epsilon = epsilon_;
	if (USE_RP) {
		this->rp_manager.set_basics(this->x_d_old.rows(), this->lambda_max, this->epsilon);
	} else {
		this->sot_manager.set_basics(this->x_d_old.rows(), this->lambda_max, this->epsilon);
	}
}

/* CHANGEHANDTYPE */
void contactPreserver::changeHandType(Eigen::MatrixXd S_) {
	// Set the new synergy matrix
	S = S_;
	ROS_DEBUG_STREAM("Changed the Synergy Matrix inside contact preserver!!!");
}

/* SETGRASPSTATE */
void contactPreserver::setGraspState(Eigen::MatrixXd J_, Eigen::MatrixXd G_,
                                     Eigen::MatrixXd T_, Eigen::MatrixXd H_) {
	// Set the new J, G, T and H matrices
	J = J_;
	G = G_;
	T = T_;
	H = H_;
}

/* SETGRASPSTATE (overloaded) */
void contactPreserver::setGraspState(Eigen::MatrixXd J_, Eigen::MatrixXd G_, Eigen::MatrixXd T_,
                   Eigen::MatrixXd H_, Eigen::MatrixXd Kc_){
    // Set the new J, G, T, H and Kcmatrices
    J = J_;
    G = G_;
    T = T_;
    H = H_;
    Kc = Kc_;
}

/* SETMINIMIZATIONPARAMS */
void contactPreserver::setMinimizationParams(Eigen::VectorXd x_d_, Eigen::VectorXd f_d_d_) {
	// Set the new desired motion vector and weight matrix
	x_d = x_d_;
	f_d_d = f_d_d_;
}

/* SETPERMUTATIONMATRIX */
void contactPreserver::setPermutationParams(Eigen::MatrixXd P_, int num_contacts_) {
	// Setting the permutation matrix
	P = P_;
	num_contacts = num_contacts_;
}

/* PERFORMKININVERSTION */
bool contactPreserver::performKinInversion(Eigen::VectorXd &x_result) {
	// Print message for debug
	if (DEBUG) std::cout << "Entered performKinInversion in ContactPreserver!" << std::endl;

	// Resize Q to be of correct size
	Q.resize(H.rows(), x_d.size());

	// Print message for debug
	if (DEBUG) std::cout << "Resized Q in contactPreserver!" << std::endl;

	// Now create the block matrix
	Q << Kc * H * J * S, Kc * H * T;

	// For debugging purposes (real line is above)
	if (N_DEBUG) {
		Eigen::FullPivLU<Eigen::MatrixXd> luN_debug(Q);
		Eigen::MatrixXd N_debug = luN_debug.kernel();
		x_result = N_debug.col(0);
		return true;
	}

	// Print message for debug
	if (DEBUG) std::cout << "Computed Q in contactPreserver!" << std::endl;

	// Now create Q_tilde by appending Q under Identity matrix
	Q_tilde.resize(x_d.size() + H.rows(), x_d.size());
	Q_tilde << Eigen::MatrixXd::Identity(x_d.size(), x_d.size()), Q;

	// Print message for debug
	if (DEBUG) std::cout << "Computed Q_tilde in contactPreserver!" << std::endl;

	// Rotating the contact wrenches to finger frames and appending
	Eigen::VectorXd f_d_d_tot = this->create_force_ref_vec(this->read_contacts_map, this->f_d_d);

	// Compute vector y
	y.resize(x_d.size() + H.rows());
	// Use also Kc
	Eigen::VectorXd y_c = f_d_d_tot + Kc * H * G.transpose() * xi_o;
	y << x_d, y_c;

	// DEBUG PRINTS
	if (DEBUG || true) {
		std::cout << "----------------" << std::endl;
		std::cout << "Q_tilde = " << Q_tilde << std::endl;
		std::cout << "y = " << y << std::endl;
		std::cout << "f_d_d = " << f_d_d << std::endl;
		std::cout << "f_d_d_tot = " << f_d_d_tot << std::endl;
		std::cout << "xi_o = " << xi_o << std::endl;
		std::cout << "y_c = " << y_c << std::endl;
		std::cout << "H*G.transpose()*xi_o = " << H * G.transpose() * xi_o << std::endl;
		std::cout << "----------------" << std::endl;
	}

	// Preparing to fill in the tasks in the RP Manager
	Eigen::VectorXd tmp_x_dot;
	Eigen::MatrixXd tmp_jacobian;
	int tmp_priority;
	int row_index;

	this->tmp_task_vec.clear();     // Clearing the vector of tasks

	// Clearing the task set
	if (USE_RP) {
		this->rp_manager.clear_set();
	} else {
		this->sot_manager.clear_set();
	}

	int dim_reached = 0;            // Temporary sum of dimension for understanding if the contacts part has been reached

	for (int i = 0; i < this->num_tasks; i++) { // Filling in the tasks except the contact ones
		// The row index is the sum of the previous dimensions
		row_index = 0;
		for (int k = 0; k < i; k++) row_index += this->dim_tasks.at(k);
		tmp_x_dot = y.block(row_index, 0, this->dim_tasks[i], y.cols());
		tmp_jacobian = Q_tilde.block(row_index, 0, this->dim_tasks[i], Q_tilde.cols());
		tmp_priority = this->prio_tasks[i];
		dim_reached += this->dim_tasks[i];

		// Push in tmp variables
		this->tmp_task.set_task_x_dot(tmp_x_dot);
		this->tmp_task.set_task_jacobian(tmp_jacobian);
		this->tmp_task.set_task_priority(tmp_priority);
		this->tmp_task_vec.push_back(tmp_task);
	}

	for (int i = 0; i < this->num_contacts; i++) { // Filling in the tasks regarding the contacts
		// Iterating and pushing back the tasks in which the contacts part is divided
		for (int j = this->num_tasks; j < this->dim_tasks.size(); j++) {
			// The row index is the sum of the previous dimensions
			tmp_x_dot = y.block(dim_reached, 0, this->dim_tasks[j], y.cols());
			tmp_jacobian = Q_tilde.block(dim_reached, 0, this->dim_tasks[j], Q_tilde.cols());
			tmp_priority = this->prio_tasks[j];
			dim_reached += this->dim_tasks[j];

			// Push in tmp variables
			this->tmp_task.set_task_x_dot(tmp_x_dot);
			this->tmp_task.set_task_jacobian(tmp_jacobian);
			this->tmp_task.set_task_priority(tmp_priority);
			this->tmp_task_vec.push_back(tmp_task);
		}
	}

	// Setting the secondary priorities (by adding +1 when first and sec priorities are same)
	for (long int i = 0; i < this->tmp_task_vec.size(); i++) {
		for (int j = i + 1; j < this->tmp_task_vec.size(); j++) {
			if (this->tmp_task_vec.at(i).get_task_priority() == this->tmp_task_vec.at(j).get_task_priority()) {
				if (this->tmp_task_vec.at(i).get_sec_priority() == this->tmp_task_vec.at(j).get_sec_priority()) {
					int curr_sec_prio = this->tmp_task_vec.at(j).get_sec_priority();
					this->tmp_task_vec.at(j).set_sec_priority(curr_sec_prio + 1);
				}
			}
		}
	}

	// Insert tasks in manager and solve
	bool solved = false;
	if (USE_RP) {
		// Pushing into RP Manager, printing out and solving
		this->rp_manager.insert_tasks(this->tmp_task_vec);
		if (DEBUG) this->rp_manager.print_set();
		solved = this->rp_manager.solve_inv_kin(x_ref);
	} else {
		// Pushing into SOT Manager, printing out and solving
		this->sot_manager.insert_tasks(this->tmp_task_vec);
		if (DEBUG) this->sot_manager.print_set();
		this->sot_manager.solve_inv_kin(x_ref);
		solved = true;
	}

	// Pass reference as solution of task inversion
	if (solved) {
		ROS_INFO_STREAM("The Task Set Solution is \n" << x_ref);
		x_ref_old = x_ref;
		x_result = x_ref;
		if (DEBUG) ROS_WARN_STREAM("A new reference has been sent! Yahoo!");
		return true;
	} else {
		ROS_ERROR("Task Inversion Manager could not find solution!");
		x_result = x_ref_old;
		return false;
	}

}

/* PRINTALL */
void contactPreserver::printAll() {
	// Print to screen the main private variables
	std::cout << "J =" << std::endl;
	std::cout << J << std::endl;
	std::cout << "G =" << std::endl;
	std::cout << G << std::endl;
	std::cout << "T =" << std::endl;
	std::cout << T << std::endl;
	std::cout << "H =" << std::endl;
	std::cout << H << std::endl;
	std::cout << "S =" << std::endl;
	std::cout << S << std::endl;
	std::cout << "x_d =" << std::endl;
	std::cout << x_d << std::endl;
	std::cout << "Q =" << std::endl;
	std::cout << Q << std::endl;
	std::cout << "N_tilde =" << std::endl;
	std::cout << N_tilde << std::endl;
}

/* SETCONTACTSMAP */
void contactPreserver::set_contacts_and_selection(std::map<int, std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>> contacts_map_, Eigen::MatrixXd H_i_){
    this->read_contacts_map = contacts_map_;
    this->H_i = H_i_;
}

/* CREATEFORCEREFVEC */
Eigen::VectorXd contactPreserver::create_force_ref_vec(std::map<int, std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>> contacts_map, Eigen::VectorXd f_d_d_in){

    int num_conts = contacts_map.size();
    int tot_size = f_d_d_in.size() * num_conts;
    int size_h_i = this->H_i.cols();

    Eigen::VectorXd tmp_f_d_d_tot(tot_size);                // Temporary vector to be filled in (declared with size)
	Eigen::VectorXd prev_f_d_d_tot(tot_size);				// For saving the above vector
    Eigen::VectorXd tmp_f_d_d_r(f_d_d_in.size());            // Temporary vector for each finger force ref
    Eigen::MatrixXd tmp_rot;
    Eigen::MatrixXd whole_rot(size_h_i, size_h_i); // 6x6 rotation matrix

    std::map<int, std::tuple<std::string, Eigen::Affine3d, Eigen::Affine3d>>::iterator it_map;
    for (it_map = contacts_map.begin(); it_map != contacts_map.end(); it_map++) {

        // Getting the rotation from finger to world
        tmp_rot = std::get<1>(it_map->second).rotation().transpose();

		ROS_INFO_STREAM("Got the contact number " << it_map->first << " and it has the rotation " << tmp_rot);

        // Building the rotation for the contact wrench variation
		whole_rot.resize(size_h_i, size_h_i);
        whole_rot.block(0,0,size_h_i/2,size_h_i/2) = tmp_rot;
        whole_rot.block(0,size_h_i/2,size_h_i/2,size_h_i/2) = Eigen::MatrixXd::Zero(3,3);
        whole_rot.block(size_h_i/2,0,size_h_i/2,size_h_i/2) = Eigen::MatrixXd::Zero(3,3);
        whole_rot.block(size_h_i/2,size_h_i/2,size_h_i/2,size_h_i/2) = tmp_rot;

		ROS_INFO_STREAM("I have built the whole rotation and it is " << whole_rot);

        // Complying with the selection matrix and rotating contact wrench
        whole_rot = this->H_i * whole_rot;
		ROS_INFO_STREAM("After mult with H_i whole rotation it is " << whole_rot);
		ROS_INFO_STREAM("f_d_d_in is " << f_d_d_in);
        tmp_f_d_d_r = whole_rot * f_d_d_in;

        // Appending
		if (it_map == contacts_map.begin()) {
			tmp_f_d_d_tot = tmp_f_d_d_r;
		} else {
			prev_f_d_d_tot = tmp_f_d_d_tot;
			tmp_f_d_d_tot.conservativeResize(prev_f_d_d_tot.size() + tmp_f_d_d_r.size());
			tmp_f_d_d_tot << prev_f_d_d_tot, tmp_f_d_d_r;
			ROS_INFO_STREAM("Appendedddd!!!!");
		}
    }

    return tmp_f_d_d_tot;

}

/* OBJECTTWISTCALLBACK */
void contactPreserver::object_twist_callback(const geometry_msgs::Twist::ConstPtr &msg) {
	// Resizing the eigen vector
	this->xi_o.resize(6);

	// Saving the msg to the eigen vector
	this->xi_o << msg->linear.x;
	this->xi_o << msg->linear.y;
	this->xi_o << msg->linear.z;
	this->xi_o << msg->angular.x;
	this->xi_o << msg->angular.y;
	this->xi_o << msg->angular.z;
}
