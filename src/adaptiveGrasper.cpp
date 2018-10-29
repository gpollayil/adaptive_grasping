#include "adaptiveGrasper.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "adaptive_grasper"

/**
* @brief The following are functions of the class adaptiveGrasper.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
adaptiveGrasper::adaptiveGrasper(){
    // Nothing to do here
}

/* OVERLOADED CONSTRUCTOR */
adaptiveGrasper::adaptiveGrasper(std::vector<std::string> param_names){
    // Building the object
    this->initialized = this->initialize(param_names);
}

/* DESTRUCTOR */
adaptiveGrasper::~adaptiveGrasper(){
    // Nothing to do here now
}

/* INITIALIZE */
bool adaptiveGrasper::initialize(std::vector<std::string> param_names){
    // Subscribe to joint states
    this->js_sub = this->ag_nh.subscribe("joint_states", 1, &adaptiveGrasper::getJointsAndComputeSyn, this);
    ROS_WARN_STREAM("adaptiveGrasper::initialize THE SUBSCRIBER SUBSCRIBED TO " << js_sub.getTopic() << ".");

    // Waiting for a message in joint states
    this->full_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", this->ag_nh);

    // Initializing the client to robot commander
    this->client_rc = this->ag_nh.serviceClient<adaptive_grasping::velCommand>("rc_service");

    // Starting to parse the needed elements from parameter server
    ROS_WARN_STREAM("adaptiveGrasper::initialize STARTING TO PARSE THE NEEDED VARIABLES!");

    this->initialized = this->ag_nh.getParam("adaptive_grasping", this->adaptive_params);
    if(this->initialized == false){
        ROS_ERROR_STREAM("adaptiveGrasper::initialize could not find the needed params");
    }
    this->initialized = this->parseParams(this->adaptive_params, param_names);

    // Building the main objects
    ROS_WARN_STREAM("adaptiveGrasper::initialize STARTING TO BUILD THE OBJECTS!");

    this->my_contact_state.intialize(this->touch_topic_name, this->link_names_map, this->params_map);
    this->my_matrices_creator.initialize(this->H_i, this->params_map.at("world_name"), this->params_map.at("palm_name"), this->joint_numbers);
    this->my_contact_preserver.initialize(this->S);

    ROS_WARN_STREAM("adaptiveGrasper::initialize FINISHED BUILDING THE OBJECTS!");
}

/* PARSEPARAMS */
bool adaptiveGrasper::parseParams(XmlRpc::XmlRpcValue params_xml, std::vector<std::string> param_names){
    // Starting to parse and save all needed single parameters
    parseParameter(params_xml, this->touch_topic_name, param_names[0]);
    parseParameter(params_xml, this->link_names_map, param_names[1]);
    parseParameter(params_xml, this->params_map, param_names[2]);
    parseParameter(params_xml, this->joint_numbers, param_names[3]);
    parseParameter(params_xml, this->H_i, param_names[4]);
    parseParameter(params_xml, this->A_tilde, param_names[5]);

    // x_d (vector) needs to be parsed differently using the parsing function for matrix
    Eigen::MatrixXd temp_x_d;
    parseParameter(params_xml, temp_x_d, param_names[6]);
    this->x_d = temp_x_d.transpose().col(0);

    parseParameter(params_xml, this->spin_rate, param_names[7]);
}

/* GETJOINTSANDCOMPUTESYN */
void adaptiveGrasper::getJointsAndComputeSyn(const sensor_msgs::JointState::ConstPtr &msg){
    // Storing the message into another global message variable
	ROS_DEBUG_STREAM("adaptiveGrasper::getJointsAndComputeSyn GOT JOINTSTATE MSG: STARTING TO SAVE!");
	this->full_joint_state = msg;
	ROS_DEBUG_STREAM("adaptiveGrasper::getJointsAndComputeSyn SAVED JOINTSTATE MSG!");

    // Using the full joint state to compute the ratios
    Eigen::MatrixXd Syn(33, 1);

    // Copying the values of the joints
    int index = find (this->full_joint_state->name.begin(),this->full_joint_state->name.end(), "right_hand_thumb_abd_joint") - this->full_joint_state->name.begin();
    	for(int j = 0; j < 33; j++){
      Syn(j) = this->full_joint_state->position[index + j];
    }

    // Dividing by synergy value to find the Matrix
    this->S = Syn / this->full_joint_state->position[index - 1];
}

/* SPINGRASPER */
void adaptiveGrasper::spinGrasper(){
    // Setting the ROS rate
    ros::Rate rate(this->spin_rate);

    // Starting the ROS loop
    while(ros::ok()){
        // Spinning once to process callbacks
        ros::spinOnce();

        // Reading the values from contact state
        this->my_contact_state.readValues(this->read_contacts_map, this->read_joints_map);

        // Setting the contacts map and joints map in creator
        
    }
}
