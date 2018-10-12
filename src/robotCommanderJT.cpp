#include "robotCommanderJT.h"

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
    std::vector<std::string> joint_names_vec_){
    // Storing the topic strings
    this->hand_topic = hand_topic_; this->arm_topic = arm_topic_;

    // Storing the joint names vector
    this->joint_names_vec = joint_names_vec_;

    // Initializing the action clients
    this->arm_client = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(this->arm_topic, true);
    this->hand_client = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(this->hand_topic, true);

    // Initializing subscriber to joint states
    this->joint_state_sub = this->nh_rc.subscribe("joint_states", 1, &robotCommanderJT::getJointStates, this);
    this->rc_jt_server = this->nh_rc.advertiseService("rc_jt_service", &robotCommanderJT::performRobotCommand, this);

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
    ROS_DEBUG_STREAM("robotCommanderJT::getJointStates GOT JOINTSTATE MSG: STARTING TO SAVE!");
    this->full_joint_state = msg;
    ROS_DEBUG_STREAM("robotCommanderJT::getJointStates SAVED JOINTSTATE MSG!");
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
        ROS_ERROR("robotCommanderJT::setKDL Failed to get robot kinematic tree!");
        return false;
    }

    // Getting chain
    if(!this->robot_kin_tree.getChain("world", "right_arm_7_link", this->robot_kin_chain)){
        ROS_ERROR("robotCommanderJT::setKDL Failed to get robot kinematic chain!");
        return false;
    }

    // Resizing variables
    this->arm_js.resize(this->robot_kin_chain.getNrOfJoints());
    this->arm_jac.resize(this->robot_kin_chain.getNrOfJoints());

    // Initializing solver
    this->jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(this->robot_kin_chain));
}

/* PERFORMROBOTCOMMAND */
bool robotCommanderJT::performRobotCommand(adaptive_grasping::velCommand::Request &req,
    adaptive_grasping::velCommand::Response &res){
    // The bool to be returned and a tmp var
    bool success;
    std::array<float, 7> x_ref_arr;

    // Saving the velocity reference in req to array and then to Eigen class variable
    std::copy(std::begin(req.x_ref), std::end(req.x_ref), std::begin(x_ref_arr));
    for(int i = 0; i < x_ref_arr.size(); i++){
        this->x_ref(i) = x_ref_arr[i];
    }
    
    ROS_INFO_STREAM("robotCommanderJT::performRobotCommand : The requested velocity is " 
        << "\n" << this->x_ref << ".");


    return success;
}
