#ifndef FULL_GRASPER_H
#define FULL_GRASPER_H

// Basic includes
#include <ros/service.h>
#include <controller_manager_msgs/SwitchController.h>
#include <eigen_conversions/eigen_msg.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"

// Custom msg and srv includes
#include "adaptive_grasping/choose_object.h"

// Custom Includes
#include "adaptiveGrasper.h"                                // Most of other h files are included in this one

/**
* @brief This class is created by the main of the full_grasping_node.
* The purpouse of this node is to start closing the hand until a touch is detected; then the
* callback of the adaptive_grasper is called.
* It also has a callback that uses panda_softhand_control package to perform a full adaptive
* grasping task, lift and handover.
*
*/

namespace adaptive_grasping {

    class fullGrasper {
        
        public:

        /** CONSTRUCTOR
        * @brief Default constructor for fullGrasper
        *
        * @param null
        * @return null
        */
        fullGrasper();

        /** OVERLOADED CONSTRUCTOR
        * @brief Default constructor for fullGrasper
        *
        * @param arm_ns_ a string containing the namespace of the arm
        * @param hand_ns_ a string containing the namespace of the hand
        * @param normal_controllers_names_ a vector containing the strings of the normal controllers
        * @param velocity_controllers_names_ a vector containing the strings of the velocity controllers
        * @return null
        */
        fullGrasper(std::string arm_ns_, std::string hand_ns_, std::vector<std::string> normal_controllers_names_, std::vector<std::string> velocity_controllers_names_);

        /** PARSETASKPARAMS
        * @brief Public function for parsing required parameters for full grasping routine
        *
        * @param null
        * @return null
        */
        bool parse_task_params();

        /** CONVERTVECTORTOPOSE
        * @brief Public function for converting xyzrpy vector to geometry_msgs Pose
        *
        * @param input_vec std vector xyzrpy
        * @return pose geometry_msgs::Pose
        */
        geometry_msgs::Pose convert_vector_to_pose(std::vector<double> input_vec);

        /** INITIALIZE
        * @brief Public function for initializing the object (recalls the initialize function of adaptive_grasper)
        *
        * @param names_vector containing the names of all parameters needed
        * @return bool = true if success
        */
        bool initialize(std::vector<std::string> param_names);

        /** SPIN
        * @brief Public function for spinning (recalls the spinGrasper function of adaptive_grasper)
        *
        * @param null
        * @return null
        */
        void spin();

        /** GETOBJECTPOSE
        * @brief Callback function for getting current object pose
        *
        * @param msg pose
        * @return null
        */
        void get_object_pose(const geometry_msgs::Pose::ConstPtr &msg);

        /** CALLSETOBJECT
        * @brief Callback function for setting the particular object to be grasped
        *
        * @param msg pose
        * @return null
        */
        bool call_set_object(adaptive_grasping::choose_object::Request &req, adaptive_grasping::choose_object::Response &res);

        /** CALLADAPTIVEGRASPTASK
        * @brief Callback function for performing the whole adaptive grasp routine
        *
        * @param req / res
        * @return bool success
        */
        bool call_adaptive_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

        /** CALLENDADAPTIVEGRASP
        * @brief Callback function for triggering the end of adaptive grasper
        *
        * @param req / res
        * @return bool success
        */
        bool call_end_adaptive_grasp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        private:

        // Rose main variables
        ros::NodeHandle nh;

        // The velocity command to be sent to the robot commander (using the one of adaptive_grasper)
        velCommand handclose_command;

        // Subscriber to object pose and the pose
        ros::Subscriber object_sub;
        geometry_msgs::Pose object_pose_T;

        // Service Servers
        ros::ServiceServer adaptive_task_server;
        ros::ServiceServer set_object_server;
        ros::ServiceServer ag_ended_server;

        // Adaptive grasping ended trigger bool
        bool adaptive_grasping_ended;

        // The XmlRpc value for parsing complex params
        XmlRpc::XmlRpcValue task_seq_params;

        // Parsed task sequence variables
        std::string arm_name;                           // Name of the robot (namespace)
        std::string hand_name;                          // Name of the hand (namespace)
        std::vector<double> home_joints;
        std::vector<double> grasp_transform;
        geometry_msgs::Pose grasp_T;
        std::vector<double> pre_grasp_transform;
        geometry_msgs::Pose pre_grasp_T;
        std::vector<double> handover_joints;
        double handover_thresh;

        std::map<std::string, std::vector<double>> poses_map;     // The map containing the notable poses

    };

}



#endif // FULL_GRASPER_H