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
    this->joint_state_sub = nh_rc.subscribe("joint_states", 1, &robotCommanderJT::getJointStates, this);

    // Setting KDL
    this->setKDL();

    // Initializing the service server of robotCommanderJT

}

/* DESTRUCTOR */
robotCommanderJT::~robotCommanderJT(){
    // Nothing to do here
}

/* GETJOINTSTATES */
void robotCommanderJT::getJointStates(const sensor_msgs::JointStateConstPtr& msg){
    // Writing the msg in the private var while using mutual exclusion
    robot_commander_mutex.lock();
    // Storing the message into another global message variable
    ROS_DEBUG_STREAM("robotCommanderJT::getJointStates GOT JOINTSTATE MSG: STARTING TO SAVE!");
    this->full_joint_state = msg;
    ROS_WARN_STREAM("robotCommanderJT::getJointStates SAVED JOINTSTATE MSG!");
    robot_commander_mutex.unlock();
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
    this->robot_kin_tree.getChain("world", "right_arm_7_link", this->robot_kin_chain);

    // Resizing variables
    this->arm_js.resize(this->robot_kin_chain.getNrOfJoints());
    this->arm_jac.resize(this->robot_kin_chain.getNrOfJoints());

    // Initializing solver
    this->jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(this->robot_kin_chain));
}
