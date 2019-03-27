#include "fullGrasper.h"
#include "utils/parsing_utilities.h"

#include "adaptive_grasping/adaptiveGrasp.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "full_grasper"

#define DEBUG_FG   0           // Prints out additional info

/**
* @brief The following are functions of the class fullGrasper.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
fullGrasper::fullGrasper(){
    // Nothing to do here
}

/* OVERLOADED CONSTRUCTOR */
fullGrasper::fullGrasper(std::string arm_ns_, std::string hand_ns_, std::vector<std::string> normal_controllers_names_, std::vector<std::string> velocity_controllers_names_){
    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }
    
    // Setting the robot controllers namespace
    this->arm_namespace = arm_ns_;
    this->hand_namespace = hand_ns_;

    // Setting the names of the controllers
    this->normal_controllers_names = normal_controllers_names_;
    this->velocity_controllers_names = velocity_controllers_names_;

    // Initializing the franka_state_sub subscriber and waiting
    this->franka_state_sub = this->nh.subscribe("/" + this->arm_name + this->franka_state_topic_name, 1, &fullGrasper::get_franka_state, this);
    ros::topic::waitForMessage<franka_msgs::FrankaState>("/" + this->arm_name + this->franka_state_topic_name, ros::Duration(2.0));

    // Initializing the tau_ext norm and franka recovery publishers
    this->pub_franka_recovery = this->nh.advertise<franka_control::ErrorRecoveryActionGoal>("/" + this->arm_name + "/franka_control/error_recovery/goal", 1);
    this->pub_tau_ext_norm = this->nh.advertise<std_msgs::Float64>("tau_ext_norm", 1);

    // Initializing Panda SoftHand Client (TODO: Return error if initialize returns false)
    this->panda_softhand_client.initialize(this->nh);

    // Initializing the object subscriber and waiting (TODO: parse the topic name)
    this->object_sub = this->nh.subscribe("/object_pose", 1, &fullGrasper::get_object_pose, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>("/object_pose", ros::Duration(2.0));

    // Advertising the services (TODO: parse the service names)
    this->adaptive_task_server = this->nh.advertiseService("adaptive_task_service", &fullGrasper::call_adaptive_grasp_task, this);
    this->set_object_server = this->nh.advertiseService("set_object_service", &fullGrasper::call_set_object, this);
    this->ag_ended_server = this->nh.advertiseService("adaptive_grasping_end_trigger", &fullGrasper::call_end_adaptive_grasp, this);

    // Initializing the service clients
    this->arm_switch_client = this->nh.serviceClient<controller_manager_msgs::SwitchController>(this->arm_name + this->switch_service_name);
    this->arm_switch_client.waitForExistence(ros::Duration(2.0));
    this->hand_switch_client = this->nh.serviceClient<controller_manager_msgs::SwitchController>(this->hand_name + this->switch_service_name);
    this->hand_switch_client.waitForExistence(ros::Duration(2.0));
}

/* PARSETASKPARAMS */
bool fullGrasper::parse_task_params(){
    bool success = true;

    if(!ros::param::get("/full_grasper/arm_name", this->arm_name)){
		ROS_WARN("The param 'arm_name' not found in param server! Using default.");
		this->arm_name = "panda_arm";
		success = false;
	}

    if(!ros::param::get("/full_grasper/hand_name", this->hand_name)){
		ROS_WARN("The param 'hand_name' not found in param server! Using default.");
		this->hand_name = "panda_joint";
		success = false;
	}

    if(!ros::param::get("/full_grasper/arm_pos_controller", this->arm_pos_controller)){
		ROS_WARN("The param 'arm_pos_controller' not found in param server! Using default.");
		this->arm_pos_controller = "position_joint_trajectory_controller";
		success = false;
	}

    if(!ros::param::get("/full_grasper/arm_vel_controller", this->arm_vel_controller)){
		ROS_WARN("The param 'arm_vel_controller' not found in param server! Using default.");
		this->arm_vel_controller = "cartesian_impedance_controller_softbots_stiff_matrix";
		success = false;
	}

    if(!ros::param::get("/full_grasper/hand_pos_controller", this->hand_pos_controller)){
		ROS_WARN("The param 'hand_pos_controller' not found in param server! Using default.");
		this->hand_pos_controller = "position_joint_trajectory_controller";
		success = false;
	}

    if(!ros::param::get("/full_grasper/hand_vel_controller", this->hand_vel_controller)){
		ROS_WARN("The param 'hand_vel_controller' not found in param server! Using default.");
		this->hand_vel_controller = "cartesian_impedance_controller_softbots_stiff_matrix";
		success = false;
	}

	if(!ros::param::get("/full_grasper/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/full_grasper/grasp_transform", this->grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform.resize(6);
        std::fill(this->grasp_transform.begin(), this->grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    if(!ros::param::get("/full_grasper/pre_grasp_transform", this->pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform.resize(6);
        std::fill(this->pre_grasp_transform.begin(), this->pre_grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform);

    if(!ros::param::get("/full_grasper/handover_joints", this->handover_joints)){
		ROS_WARN("The param 'handover_joints' not found in param server! Using default.");
		this->handover_joints = {-0.101, 0.161, 0.159, -1.651, 2.023, 2.419, -0.006};
		success = false;
	}

    if(!ros::param::get("/full_grasper/handover_thresh", this->handover_thresh)){
		ROS_WARN("The param 'handover_thresh' not found in param server! Using default.");
		this->handover_thresh = 4.5;
		success = false;
	}

    // Getting the XmlRpc value and parsing
    if(!ros::param::get("/full_grasper", this->task_seq_params)){
        ROS_ERROR("Could not get the XmlRpc value.");
        success = false;
    }

    if(!parseParameter(this->task_seq_params, this->poses_map, "poses_map")){
        ROS_ERROR("Could not parse the poses map.");
        success = false;
    }

    if(DEBUG_FG){
        ROS_INFO_STREAM("The poses map is");
        for(auto it : this->poses_map){
            std::cout << it.first << " : [ ";
            for(auto vec_it : it.second){
                std::cout << vec_it << " ";
            } 
            std::cout << "]" << std::endl;     
        }
    }

    return success;
}

/* CONVERTVECTORTOPOSE */
geometry_msgs::Pose fullGrasper::convert_vector_to_pose(std::vector<double> input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);
    return output_pose;
}

/* INITIALIZE */
bool fullGrasper::initialize(std::vector<std::string> param_names){
    // Nothing to do here
}

/* SWITCHCONTROL */
bool fullGrasper::switch_control(std::string robot_name, std::string from_controller, std::string to_controller){
    // Temporary bool to be returned
    bool success = false;

    // Clearing the switch message
    this->switch_controller.request.start_controllers.clear();
    this->switch_controller.request.stop_controllers.clear();

    // Filling up the switch message
    this->switch_controller.request.start_controllers.push_back(to_controller);
    this->switch_controller.request.stop_controllers.push_back(from_controller);
    this->switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

    // Swithching controller by calling the service
    if(robot_name == this->arm_name){
        this->arm_switch_client.waitForExistence(ros::Duration(5.0));
        return this->arm_switch_client.call(this->switch_controller);
    } else if (robot_name == this->hand_name){
        this->hand_switch_client.waitForExistence(ros::Duration(5.0));
        return this->hand_switch_client.call(this->switch_controller);
    }

    ROS_ERROR("fullGrasper : in switch_control unknown robot name!");
    return false;
}

/* SPIN */
void fullGrasper::spin(){
    // Nothing to do here
}

/* GETOBJECTPOSE */
void fullGrasper::get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T = *msg;
}

/* CALLSETOBJECT */
bool fullGrasper::call_set_object(adaptive_grasping::choose_object::Request &req, adaptive_grasping::choose_object::Response &res){

    // Checking if the parsed map contains the requested object
    auto search = this->poses_map.find(req.object_name);
    if(search == this->poses_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }

    // Setting the grasp pose as requested
    this->grasp_transform = this->poses_map.at(req.object_name);

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    // Now, everything is ok
    ROS_INFO_STREAM("Grasp pose changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;
}

/* GETFRANKASTATE */
void fullGrasper::get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg){

    // Saving the message
    this->latest_franka_state = *msg;

    // Checking for libfranka errors
    if(msg->robot_mode != 2 && msg->robot_mode != 5){       // The robot state is not "automatic" or "manual guiding"
        this->franka_ok = false;
        if(DEBUG_FG && false) ROS_ERROR("Something happened to the robot!");
    }else if(msg->robot_mode == 2){
        this->franka_ok = true;
        if(DEBUG_FG && false) ROS_WARN("Now Franka is in a good mood!");
    }

    // Getting the tau ext
    if(DEBUG_FG && false){
        std::cout << "The latest tau ext vector is \n [ ";
        for(auto it : this->latest_franka_state.tau_ext_hat_filtered)  std::cout << it << " ";
        std::cout << "]" << std::endl;
    }

    // Computing the norm
    this->tau_ext_norm = 0.0;
    for(auto it : this->latest_franka_state.tau_ext_hat_filtered){
        this->tau_ext_norm += std::pow(it, 2);
    }
    this->tau_ext_norm = std::sqrt(this->tau_ext_norm);

    // Publishing norm
    std_msgs::Float64 norm_msg; norm_msg.data = this->tau_ext_norm;
    this->pub_tau_ext_norm.publish(norm_msg);
    
}

/* CALLADAPTIVEGRASPTASK */
bool fullGrasper::call_adaptive_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_adaptive_grasp_task done correctly with false request!";
        return true;
    }

    // 1) Going to home configuration
    if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified home joint configuration.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    sleep(2);       // TODO: listen to robot and make sure it's not moving

    // 4.1) Switching to velocity controllers
    if(!this->switch_control(this->arm_name, this->arm_pos_controller, this->arm_vel_controller) || !this->franka_ok){
        ROS_ERROR_STREAM("Could not switch to the arm velocity controller " 
            << this->arm_vel_controller << " from " << this->arm_pos_controller << ". Are these controllers loaded?");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }
    sleep(1);     // Giving some time to controller manager
    if(!this->switch_control(this->hand_name, this->hand_pos_controller, this->hand_vel_controller) || !this->franka_ok){
        ROS_ERROR_STREAM("Could not switch to the hand velocity controller " 
            << this->hand_vel_controller << " from " << this->hand_pos_controller << ". Are these controllers loaded?");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // Calling the adaptive grasp service and waiting for its completion
    this->adaptive_grasping_ended = false;                                  // Setting end trigger to false

    adaptive_grasping::adaptiveGrasp ag_srv; ag_srv.request.run_adaptive_grasp = true;
    if(!ros::service::call<adaptive_grasping::adaptiveGrasp>("adaptive_grasper_service", ag_srv)){
        ROS_ERROR("Could not call the adaptive_grasper_service.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    ROS_INFO("Called the adaptive_grasper_service!");

    // 4) Wait for an event on a topic
    while(!this->adaptive_grasping_ended){
        // Sleep for some time
        usleep(50);
    }

    ROS_INFO("Someone triggered the adaptive grasp end!");

    sleep(2);       // TODO: listen to robot and make sure it's not moving

    // 4.4) Switching to back to position controllers
    if(!this->switch_control(this->arm_name, this->arm_vel_controller, this->arm_pos_controller) || !this->franka_ok){
        ROS_ERROR_STREAM("Could not switch to the arm position controller " 
            << this->arm_pos_controller << " from " << this->arm_vel_controller << ". Are these controllers loaded?");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }
    sleep(1);     // Giving some time to controller manager
    if(!this->switch_control(this->hand_name, this->hand_vel_controller, this->hand_pos_controller) || !this->franka_ok){
        ROS_ERROR_STREAM("Could not switch to the hand position controller " 
            << this->hand_pos_controller << " from " << this->hand_vel_controller << ". Are these controllers loaded?");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 4.5) Completing grasp
    if(!this->panda_softhand_client.call_hand_service(1.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not complete the grasp.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 5) Lifting up the object
    if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 6) Going to handover joint config
    if(!this->panda_softhand_client.call_joint_service(this->handover_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified handover joint config.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 7) Waiting for threshold or for some time
    sleep(1);       // Sleeping for a second to avoid robot stopping peaks
    bool hand_open = false; ros::Time init_time = ros::Time::now(); ros::Time now_time;
    double base_tau_ext = this->tau_ext_norm;           // Saving the present tau for later computation of variation
    while(!hand_open){
        now_time = ros::Time::now();
        usleep(500);                         // Don't know why, but the threshold works with this sleeping
        if(DEBUG_FG) ROS_WARN_STREAM("The tau_ext difference is " << std::abs(this->tau_ext_norm - base_tau_ext) << " and the threshold is " << this->handover_thresh << ".");

        if(std::abs(this->tau_ext_norm - base_tau_ext) > this->handover_thresh){
            hand_open = true;
            if(DEBUG_FG) ROS_WARN_STREAM("Opening condition reached!" << " SOMEONE PULLED!");
            if(DEBUG_FG) ROS_WARN_STREAM("The tau_ext difference is " << std::abs(this->tau_ext_norm - base_tau_ext) << " and the threshold is " << this->handover_thresh << ".");
        }
        if((now_time - init_time) > ros::Duration(10, 0)){
            hand_open = true;
            if(DEBUG_FG) ROS_WARN_STREAM("Opening condition reached!" << " TIMEOUT!");
            if(DEBUG_FG) ROS_WARN_STREAM("The initial time was " << init_time << ", now it is " << now_time 
                << ", the difference is " << (now_time - init_time) << " and the timeout thresh is " << ros::Duration(10, 0));
        }
    }

    // 8) Opening hand 
    if(!this->panda_softhand_client.call_hand_service(0.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not open the hand.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_adaptive_grasp_task was correctly performed!";
    return true;
}

/* CALLENDADAPTIVEGRASP */
bool fullGrasper::call_end_adaptive_grasp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    // Setting the trigger bool
    this->adaptive_grasping_ended = true;
}