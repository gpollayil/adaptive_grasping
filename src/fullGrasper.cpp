#include "fullGrasper.h"
#include "utils/parsing_utilities.h"

#define EXEC_NAMESPACE    "adaptive_grasping"
#define CLASS_NAMESPACE   "full_grasper"

#define DEBUG   0           // Prints out additional info (additional to ROS_DEBUG)

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

    // Initializing the object subscriber and waiting (TODO: parse the topic name)
    this->object_sub = this->nh.subscribe("/object_pose", 1, &fullGrasper::get_object_pose, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>("/object_pose", ros::Duration(2.0));
}

/* PARSETASKPARAMS */
bool fullGrasper::parse_task_params(){
    bool success = true;

    if(!ros::param::get("/task_sequencer/robot_name", this->robot_name)){
		ROS_WARN("The param 'robot_name' not found in param server! Using default.");
		this->robot_name = "panda_arm";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/robot_joints_name", this->robot_joints_name)){
		ROS_WARN("The param 'robot_joints_name' not found in param server! Using default.");
		this->robot_joints_name = "panda_joint";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/pos_controller", this->pos_controller)){
		ROS_WARN("The param 'pos_controller' not found in param server! Using default.");
		this->pos_controller = "position_joint_trajectory_controller";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/imp_controller", this->imp_controller)){
		ROS_WARN("The param 'imp_controller' not found in param server! Using default.");
		this->imp_controller = "cartesian_impedance_controller_softbots_stiff_matrix";
		success = false;
	}

	if(!ros::param::get("/task_sequencer/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/grasp_transform", this->grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform.resize(6);
        std::fill(this->grasp_transform.begin(), this->grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    if(!ros::param::get("/task_sequencer/pre_grasp_transform", this->pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform.resize(6);
        std::fill(this->pre_grasp_transform.begin(), this->pre_grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform);

    if(!ros::param::get("/task_sequencer/handover_joints", this->handover_joints)){
		ROS_WARN("The param 'handover_joints' not found in param server! Using default.");
		this->handover_joints = {-0.101, 0.161, 0.159, -1.651, 2.023, 2.419, -0.006};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/handover_thresh", this->handover_thresh)){
		ROS_WARN("The param 'handover_thresh' not found in param server! Using default.");
		this->handover_thresh = 4.5;
		success = false;
	}

    // Getting the XmlRpc value and parsing
    if(!ros::param::get("/task_sequencer", this->task_seq_params)){
        ROS_ERROR("Could not get the XmlRpc value.");
        success = false;
    }

    if(!parseParameter(this->task_seq_params, this->poses_map, "poses_map")){
        ROS_ERROR("Could not parse the poses map.");
        success = false;
    }

    if(DEBUG){
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
    // Switching controllers to velocity and twist
    // this->switch_control(this->arm_namespace, this->normal_controllers_names[0], this->velocity_controllers_names[0]);
    // this->switch_control(this->hand_namespace, this->normal_controllers_names[1], this->velocity_controllers_names[1]);

    // Using the initialize of adaptiveGrasper
    if(!this->adaptive_grasper.initialize(param_names)) return false;
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
    this->switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    // Swithching controller by calling the service
    return ros::service::call<controller_manager_msgs::SwitchController>(robot_name + this->switch_service_name, this->switch_controller);
}

/* SPIN */
void fullGrasper::spin(){
    // Spinning the grasper
    this->adaptive_grasper.spinGrasper();
}

/* GETOBJECTPOSE */
void fullGrasper::get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T = *msg;
}

/* CALLSETOBJECT */
bool fullGrasper::call_set_object(adaptive_grasping::set_object::Request &req, adaptive_grasping::set_object::Response &res){

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
