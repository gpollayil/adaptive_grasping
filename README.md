# adaptive_grasping

A ROS Package for the Touch-Based Adaptive Grasping strategy.

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

This package depends on ROS Kinetic or newer.

Needed Packages (from Centro Piaggio GitHub):
`franka_ros` (branch ?), `panda-softhand` (branch ?), `pisa-iit-soft-hand` (branch ?), `IMU` (NMMI GitHub), `imu_glove_finger_touch_utils` (branch ?)

### Installing

To install this package just clone into your catkin_ws and catkin_make.

## Running the Adaptive Grasping Package

(If REAL ROBOT remember to set correctly robot_ip and set load_gripper to false in launchPandaSoftHand.launch)

(Most of the parameters of Adaptive Grasping can be changed from adaptive_params.yaml)

### Launch Order (Temporary)

1. `roslaunch adaptive_grasping launchLWRSoftHandTwist.launch` (Launches Robot in RViz + Gazebo / Real Robot)
2. `rosrun adaptive_grasping adaptive_grasping_robotCommander` (Sends the commands given by the adaptive_grasping_node to the robot controllers)
3. `rosrun adaptive_grasping adaptive_grasping_node` (The main package which updates the state of contacts, computes the main matrices and performs minimization for finding the references to be sent to robotCommander)
4. `rosrun finger_fk finger_joints_service` (Needed by contactState of adaptive_grasping_node for getting the states of the fingers)
5. (TODO: SHOULD BE GIVEN BY VISION IN FUTURE) `rostopic pub /object_pose_topic geometry_msgs/Pose "position: ... orientation: ..."`
6. `rostopic pub -r 50 /touching_finger_topic std_msgs/Int8 "data: 4"` (From 0 to 5 for fingers from thumb to pinky)
7. `rosservice call /adaptive_grasper_service "run_adaptive_grasp: true"` (To start the grasping)


