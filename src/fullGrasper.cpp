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

    // Initializing the object subscriber and waiting (TODO: parse the topic name)
    this->object_sub = this->nh.subscribe("/object_pose", 1, &fullGrasper::get_object_pose, this);
    // Commenting the wait for message for Klampt
    // ros::topic::waitForMessage<geometry_msgs::Pose>("/object_pose", ros::Duration(2.0));

    // Advertising the services (TODO: parse the service names)
    this->adaptive_task_server = this->nh.advertiseService("adaptive_task_service", &fullGrasper::call_adaptive_grasp_task, this);
    this->set_object_server = this->nh.advertiseService("set_object_service", &fullGrasper::call_set_object, this);
    this->ag_ended_server = this->nh.advertiseService("adaptive_grasping_end_trigger", &fullGrasper::call_end_adaptive_grasp, this);
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

/* CALLADAPTIVEGRASPTASK */
bool fullGrasper::call_adaptive_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_adaptive_grasp_task done correctly with false request!";
        return true;
    }

	sleep(2);       // TODO: listen to robot and make sure it's not moving

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

    // TODO: HERE IMPLEMENT THE LIFTING FOR KLAMPT

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