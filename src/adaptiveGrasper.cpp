#include "adaptiveGrasper.h"
#include <eigen_conversions/eigen_msg.h>

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "adaptive_grasper"

#define DEBUG   0           // Prints out additional info (additional to ROS_DEBUG)

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
    ROS_INFO_STREAM("adaptiveGrasper::initialize A SUBSCRIBER SUBSCRIBED TO " << js_sub.getTopic() << ".");

    // Waiting for a message in joint states
    this->full_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", this->ag_nh);

    // Initializing the client to robot commander
    this->client_rc = this->ag_nh.serviceClient<adaptive_grasping::velCommand>("rc_service");

    // Starting to parse the needed elements from parameter server
    ROS_INFO_STREAM("adaptiveGrasper::initialize STARTING TO PARSE THE NEEDED VARIABLES!");

    this->initialized = this->ag_nh.getParam("adaptive_grasping", this->adaptive_params);
    if(this->initialized == false){
        ROS_ERROR_STREAM("adaptiveGrasper::initialize could not find the needed params");
    }
    this->initialized = this->parseParams(this->adaptive_params, param_names);

    // Subscribing to object pose
    this->op_sub = this->ag_nh.subscribe(this->object_topic_name, 1, &adaptiveGrasper::getObjectPose, this);
    ROS_INFO_STREAM("adaptiveGrasper::initialize A SUBSCRIBER SUBSCRIBED TO " << op_sub.getTopic() << ".");

    // Waiting for a message in object pose
    geometry_msgs::Pose::ConstPtr tmp_op = ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name, this->ag_nh);

    // Building the main objects
    ROS_INFO_STREAM("adaptiveGrasper::initialize STARTING TO BUILD THE OBJECTS!");

    this->my_contact_state.intialize(this->touch_topic_name, this->link_names_map, this->params_map);
    this->my_matrices_creator.initialize(this->H_i, this->params_map.at("world_name"), this->params_map.at("palm_name"), this->joint_numbers);
    this->my_contact_preserver.initialize(this->S);

    ROS_INFO_STREAM("adaptiveGrasper::initialize FINISHED BUILDING THE OBJECTS!");
}

/* PRINTPARSED */
void adaptiveGrasper::printParsed(){
    ROS_INFO_STREAM("\nThe finger touch topic is: " << this->touch_topic_name << ".");
    ROS_INFO_STREAM("\nThe map with link names is:");
    for(auto it : this->link_names_map){
        std::cout << it.first << " : " << it.second << std::endl;
    }
    ROS_INFO_STREAM("\nThe map with params names is:");
    for(auto it : this->params_map){
        std::cout << it.first << " : " << it.second << std::endl;
    }
    ROS_INFO_STREAM("\nThe array of joint finger link no. is:");
    std::cout << "[ ";
    for(auto it : this->joint_numbers){
        std::cout << it << " ";
    }
    std::cout << "]" << std::endl;
    ROS_INFO_STREAM("\nThe contact selection matrix H is: \n" << this->H_i << ".");
    ROS_INFO_STREAM("\nThe min. weight matrix A tilde is: \n" << this->A_tilde << ".");
    ROS_INFO_STREAM("\nThe desired motion x_d is: \n" << this->x_d << ".");
    ROS_INFO_STREAM("\nThe object pose topic is: " << this->object_topic_name << ".");
    ROS_INFO_STREAM("\nThe reference scaling factor is: " << this->scaling << ".");
}

/* PRINTCONTACTSINFO */
void adaptiveGrasper::printContactsInfo(){
    ROS_INFO_STREAM("\nThe map with contacts info is: \n");
    for(auto it : this->read_contacts_map){
        std::cout << it.first << " : \n" << std::get<0>(it.second) << " - " << std::get<1>(it.second).matrix() << " - " << std::get<2>(it.second).matrix() << std::endl;
    }
    ROS_INFO_STREAM("\nThe map with joints info is: \n");
    for(auto it : this->read_joints_map){
        std::cout << it.first << " : " << it.second << std::endl;
    }
}

/* PRINTOBJECTPOSE */
void adaptiveGrasper::printObjectPose(){
    ROS_INFO_STREAM("\nThe object pose is: \n" << this->object_pose.matrix() << ".");
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
    parseParameter(params_xml, this->object_topic_name, param_names[8]);
    parseParameter(params_xml, this->scaling, param_names[9]);
}

/* SETCOMMANDANDSEND */
bool adaptiveGrasper::setCommandAndSend(Eigen::VectorXd ref_vec, adaptive_grasping::velCommand comm){
    // Clearing the previous service file
    comm.request.x_ref.clear();
    
    // Filling up the request
    comm.request.x_ref.push_back(ref_vec(0));
    comm.request.x_ref.push_back(ref_vec(1));
    comm.request.x_ref.push_back(ref_vec(2));
    comm.request.x_ref.push_back(ref_vec(3));
    comm.request.x_ref.push_back(ref_vec(4));
    comm.request.x_ref.push_back(ref_vec(5));
    comm.request.x_ref.push_back(ref_vec(6));

    if(this->client_rc.call(comm)){
      if(DEBUG) ROS_INFO_STREAM("adaptiveGrasper::setCommandAndSend Success!");
      return true;
    } else {
      if(DEBUG) ROS_INFO_STREAM("adaptiveGrasper::setCommandAndSend Failed!");
      return false;
    }
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
    if(DEBUG) std::cout << "********************************* " << std::endl;
    for(int j = 0; j < 33; j++){
        if(DEBUG) std::cout << "Getting the joint " << this->full_joint_state->name[index + j] << " with value " << this->full_joint_state->position[index + j] << "." << std::endl;
        Syn(j) = this->full_joint_state->position[index + j];
    }
    if(DEBUG) std::cout << "********************************* " << std::endl;

    // Dividing by synergy value to find the Matrix
    this->adaptive_grasper_mutex.lock();
    this->S = Syn / this->full_joint_state->position[index - 1];
    this->adaptive_grasper_mutex.unlock();
}

/* GETOBJECTPOSE */
void adaptiveGrasper::getObjectPose(const geometry_msgs::Pose::ConstPtr &msg){
    // Saving to the eigen affine
    this->adaptive_grasper_mutex.lock();
    tf::poseMsgToEigen(*msg, this->object_pose);
    this->adaptive_grasper_mutex.unlock();
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

        // Printing contacts info and synergy matrix
        if(DEBUG) ROS_INFO_STREAM("\nSynergy Matrix S: \n" << this->S << ".\n");
        if(DEBUG) this->printContactsInfo();
        if(DEBUG) this->printObjectPose();

        // Setting the necessary things in matrix creator and computing matrices
        this->my_matrices_creator.setContactsMap(this->read_contacts_map);
        this->my_matrices_creator.setJointsMap(this->read_joints_map);
        this->my_matrices_creator.setObjectPose(this->object_pose);
        this->my_matrices_creator.computeAllMatrices();

        // Reading and couting the matrices
        this->my_matrices_creator.readAllMatrices(this->read_J, this->read_G, this->read_T, this->read_H);
        if(DEBUG){
            ROS_INFO_STREAM("adaptiveGrasper::spinGrasper The created matrices are: ");
            ROS_INFO_STREAM("\nJ = " << "\n" << this->read_J << "\n");
            ROS_INFO_STREAM("\nG = " << "\n" << this->read_G << "\n");
            ROS_INFO_STREAM("\nT = " << "\n" << this->read_T << "\n");
            ROS_INFO_STREAM("\nH = " << "\n" << this->read_H << "\n");
        }

        // Setting the synergy matrix in preserver
        this->my_contact_preserver.changeHandType(this->S);

        // Resetting the reference motion to zero
        this->x_ref = Eigen::VectorXd::Zero(this->x_d.size());

        // Performing the minimization only if there are contacts (i.e. the matrices are not empty)
        if(read_J.innerSize() > 0 && read_G.innerSize() > 0 && read_T.innerSize() > 0 && read_H.innerSize() > 0){
            // Setting grasp state
            this->my_contact_preserver.setGraspState(this->read_J, this->read_G, this->read_T, this->read_H);

            // Setting minimization parameters
            this->my_contact_preserver.setMinimizationParams(this->x_d, this->A_tilde);

            // Performing minimization
            this->x_ref = this->my_contact_preserver.performMinimization();

            if(DEBUG) ROS_DEBUG_STREAM("adaptiveGrasper::spinGrasper Performed Minimization!!!");
        }

        // Scaling the reference and sending to the robot commander
        this->x_ref = this->scaling * this->x_ref;
        if(true) ROS_INFO_STREAM("The reference to be sent to the commander is: \n" << this->x_ref << ".");

        if(!this->setCommandAndSend(this->x_ref, this->ref_command)){
            ROS_ERROR_STREAM("adaptiveGrasper::spinGrasper Something went wrong while sending the reference to the commander!");
        }

        // Rate
        rate.sleep();
    }
}
