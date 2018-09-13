/*
    tf_listener.cpp

    Purpose: A ROS node listener and publisher for frames.

    Input: frame_name, topic_name
    Outputs the x,y,z and r,p,y of the frame in the given topic

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

// ROS INCLUDES
#include <ros/ros.h>

// TF INCLUDES
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define DEBUG			0										// if DEBUG 1 prints additional couts

// GLOBAL VARIABLES
tf::StampedTransform transform;             // for storing the listened tf
ros::Publisher pub_tf_info; 								// publisher for transformed tf

// AUX FUNCTION
void transform_and_publish(const tf::StampedTransform& trans) {
  geometry_msgs::PoseStamped trans_msg;
  tf::poseStampedTFToMsg(trans, trans_msg);               // converting
  pub_tf_info.publish(trans_msg);
}

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "tf_listen_publish_node");
	ros::NodeHandle tflp_nh;

  // Creating a ROS publisher for the output
	pub_tf_info = tflp_nh.advertise<geometry_msgs::PoseStamped>("output_topic", 1);

	// Creating a TF listener and listening
  tf::TransformListener tf_listener;
  ros::Rate rate(10.0);

  while (tflp_nh.ok()){
    try{
      tf_listener.lookupTransform("/right_hand_ee_link", "/world", ros::Time(0), transform);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  transform_and_publish(transform);

	// Success message
	std::cout << "TF Listener started to work! The TF info are being published" << std::endl;

	// Spin
	ros::spin ();

}
