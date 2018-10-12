#include "robotCommanderJT.h"
#include <iterator>

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "robot_commander_jt"
#define DEBUG             0   // print out additional info

/**
* @brief The following are functions of the class robotCommanderJT.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
robotCommanderJT::robotCommanderJT(std::string hand_topic_, std::string arm_topic_,
    std::vector<std::string> joint_names_vec_, ros::Duration header_dur_, 
      ros::Duration exec_wait_dur_, ros::Duration dt_){
    // Storing the topic strings
    this->hand_topic = hand_topic_; this->arm_topic = arm_topic_;

    // Storing the joint names vector
    this->joint_names_vec = joint_names_vec_;

    // Setting the integration time
    this->header_dur = header_dur_;
    this->exec_wait_dur = exec_wait_dur_;
    this->dt = dt_;

    // Initializing the action clients
    this->arm_client = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(this->arm_topic, true);
    this->hand_client = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(this->hand_topic, true);

    // Initializing subscriber to joint states
    this->joint_state_sub = this->nh_rc.subscribe("joint_states", 1, &robotCommanderJT::getJointStates, this);
    this->rc_jt_server = this->nh_rc.advertiseService("rc_jt_service", &robotCommanderJT::performRobotCommand, this);

    // Setting the joint names for the trajectory messages (arm first and then hand)
    for(int j = 0; j < this->joint_names_vec.size() - 1; j++){
        this->arm_traj.joint_names.push_back(this->joint_names_vec[j]);
    }
    this->hand_traj.joint_names.push_back(this->joint_names_vec.back());

    // Resizing the Eigen Vectors
    x_ref.resize(7);

    // Setting KDL
    this->setKDL();

    // Initializing the service server of robotCommanderJT

}

/* DESTRUCTOR */
robotCommanderJT::~robotCommanderJT(){
    // Shutting down subscriber and action clients
    this->joint_state_sub.shutdown();
}

/* GETJOINTSTATES */
void robotCommanderJT::getJointStates(const sensor_msgs::JointStateConstPtr& msg){
    // Writing the msg in the private var while using mutual exclusion
    this->robot_commander_mutex.lock();
    // Storing the message into other variable
    ROS_DEBUG_STREAM("robotCommanderJT::getJointStates : GOT JOINTSTATE MSG: STARTING TO SAVE!");
    this->full_joint_state = msg;
    ROS_DEBUG_STREAM("robotCommanderJT::getJointStates : SAVED JOINTSTATE MSG!");
    this->robot_commander_mutex.unlock();

    // Saving the needed joints (in joint_names_vec) to the current_joints_vector
    this->current_joints_vector = this->extractJoints();
}

/* EXTRACTJOINTS */
Eigen::VectorXd robotCommanderJT::extractJoints(){
    // Creating the vector to be returned and the index to be used for extracting data
    Eigen::VectorXd vector_js(8);
    int index;

    // Finding the states of arm and hand and pushing back using joint_names_vec
    for(int j = 0; j < this->joint_names_vec.size(); j++){
        index = find (full_joint_state->name.begin(),full_joint_state->name.end(), this->joint_names_vec[j]) - full_joint_state->name.begin();
	        vector_js(j) = full_joint_state->position[index];
    }

    return vector_js;
}

/* SETKDL */
bool robotCommanderJT::setKDL(){
    // Load robot description from ROS parameter server
    std::string robot_description_string;
    nh_rc.param("robot_description", robot_description_string, std::string());

    // Get kinematic tree from robot description
    if (!kdl_parser::treeFromString(robot_description_string, this->robot_kin_tree)){
        ROS_ERROR("robotCommanderJT::setKDL : Failed to get robot kinematic tree!");
        return false;
    }

    // Getting chain
    if(!this->robot_kin_tree.getChain("world", "right_arm_7_link", this->robot_kin_chain)){
        ROS_ERROR("robotCommanderJT::setKDL : Failed to get robot kinematic chain!");
        return false;
    }
    
    // Checking if length of joint_names_vec is equal to chain joints number + 1 (synergy joint)
    if(this->joint_names_vec.size() != this->robot_kin_chain.getNrOfJoints() + 1){
        ROS_ERROR("robotCommanderJT::setKDL : The number of joint names and the number of joints in chain are different!");
        return false;
    }

    // Resizing variables
    this->arm_js.resize(this->robot_kin_chain.getNrOfJoints());
    this->arm_jac.resize(this->robot_kin_chain.getNrOfJoints());
    joint_limits.min.resize(this->joint_names_vec.size());
    joint_limits.max.resize(this->joint_names_vec.size());
    joint_limits.velocity.resize(this->joint_names_vec.size());

    // Initializing solver
    this->jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(this->robot_kin_chain));

    // Parsing the URDF model
    if(!this->urdf.initParamWithNodeHandle("robot_description", nh_rc)){
        ROS_ERROR("robotCommanderJT::setKDL : Failed to get the URDF from robot description!");
        return false;
    }

    // For each joint in joint_names_vec getting the joint limits
    for(int j = 0; j < this->joint_names_vec.size(); j++){
        // Setting the URDF joint using the joint name and checking if it is present in the URDF model
        this->urdf_joint = this->urdf.getJoint(this->joint_names_vec[j]);
        if(!this->urdf_joint){
            ROS_ERROR("robotCommanderJT::setKDL : Could not find joint '%s' in the URDF.", this->joint_names_vec[j].c_str());
        }

        // Getting and setting the joint limits
        this->joint_limits.min(j) = this->urdf_joint->limits->lower;
        this->joint_limits.max(j) = this->urdf_joint->limits->upper;
        this->joint_limits.velocity(j) = this->urdf_joint->limits->velocity;
    }

    return true;
}

/* PERFORMROBOTCOMMAND */
bool robotCommanderJT::performRobotCommand(adaptive_grasping::velCommand::Request &req,
    adaptive_grasping::velCommand::Response &res){
    // The bool to be returned and a tmp var
    bool success = true;
    std::array<float, 7> x_ref_arr;

    // Saving the velocity reference in req to array and then to Eigen class variable
    std::copy(std::begin(req.x_ref), std::end(req.x_ref), std::begin(x_ref_arr));
    for(int i = 0; i < x_ref_arr.size(); i++){
        this->x_ref(i) = x_ref_arr[i];
    }
    
    ROS_INFO_STREAM("robotCommanderJT::performRobotCommand : The requested velocity vector is:" 
        << "\n" << this->x_ref << ".");

    // Performing inverse differential kinematics with x_ref of arm to find q_dots
    this->arm_js.data = this->current_joints_vector.head(7);
    this->jnt_to_jac_solver->JntToJac(this->arm_js, this->arm_jac);
    pseudo_inverse(this->arm_jac.data, this->arm_jac_pinv);

    // Getting required arm joint velocities
    this->q_dot_arm = this->arm_jac_pinv * this->x_ref.head(6);
    this->q_dot_hand = this->x_ref(6);

    // Saturating the joint velocities using the limits
    // (TODO with scaling)

    // Integrating and executing
    success = integrateAndExecute();

    // Return the callback result
    res.success = success;
    return success;
}

/* INTEGRATEANDEXECUTE */
bool robotCommanderJT::integrateAndExecute(){
    // The result to be returned
    bool result = true;

    // FOR DEBUGGING
    ROS_INFO_STREAM("robotCommanderJT::performRobotCommand : The current_joints_vector:" 
        << "\n" << this->current_joints_vector << ".");
    ROS_INFO_STREAM("robotCommanderJT::performRobotCommand : The q_dot_arm:" 
        << "\n" << this->q_dot_arm << ".");
    ROS_INFO_STREAM("robotCommanderJT::performRobotCommand : The q_dot_hand:" 
        << "\n" << this->q_dot_hand << ".");

    // Integrating arm velocities and creating arm trajectory
    this->arm_traj.points.clear();
    this->tmp_point.positions.clear();
    for(int j = 0; j < this->robot_kin_chain.getNrOfJoints(); j++){
        this->tmp_point.positions.push_back(this->current_joints_vector(j) + 
            this->q_dot_arm(j) * this->dt.toSec());
    }
    this->tmp_point.time_from_start = this->dt;
    this->arm_traj.points.push_back(this->tmp_point);

    ROS_INFO_STREAM("robotCommanderJT::performRobotCommand : The arm joints to be sent are:" 
        << "\n" << this->current_joints_vector + this->q_dot_arm * this->dt.toSec() << ".");

    // Integrating hand velocities and creating hand trajectory
    this->hand_traj.points.clear();
    this->tmp_point.positions.clear();
    this->tmp_point.positions.push_back(this->current_joints_vector(7) + 
            this->q_dot_hand * this->dt.toSec());
    this->tmp_point.time_from_start = this->dt;
    this->hand_traj.points.push_back(this->tmp_point);

    ROS_INFO_STREAM("robotCommanderJT::performRobotCommand : The hand joint to be sent are:" 
        << "\n" << this->current_joints_vector(7) + this->q_dot_hand * this->dt.toSec() << ".");

    // Setting the headers for both traj messages
    this->arm_traj.header.stamp = ros::Time::now() + this->header_dur;
    this->hand_traj.header.stamp = ros::Time::now() + this->header_dur;

    // Setting the goalmessages
    this->arm_goalmsg.trajectory = this->arm_traj;
    this->hand_goalmsg.trajectory = this->hand_traj;

    // Publishing to joint traj controllers
    this->arm_client->sendGoal(this->arm_goalmsg);
    this->hand_client->sendGoal(this->hand_goalmsg);

    if(!this->arm_client->waitForResult(this->exec_wait_dur)){
        ROS_WARN_STREAM("The arm is taking too much time to close! \n");
        result = false;
  	}
    if(!this->hand_client->waitForResult(this->exec_wait_dur)){
  		ROS_WARN_STREAM("The hand is taking too much time to close! \n");
        result = false;
  	}
    
    return result;

}
