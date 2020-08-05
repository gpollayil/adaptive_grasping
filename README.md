# adaptive_grasping

A ROS Package for the Sequential Contact-Based Adaptive Grasping Algorithm.

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

This package depends on ROS Kinetic or newer.

Needed Packages (from Centro Piaggio GitHub):
`franka_ros` (branch `soma-devel`), `panda-softhand` (branch `master`), `pisa-iit-soft-hand` (branch `demo-march-2019`), `IMU` (NMMI GitHub), `imu_glove_finger_touch_utils` (branch `demo-march-2019`), `finger_fk` (branch `demo-march-2019`)

### Installing

To install this package just clone into your catkin_ws and catkin_make.

## Running Adaptive Grasping

### Real Robot (Franka Emika Panda)

Remember to set correctly `robot_ip` and set `load_gripper` to `false` in `launchPandaSoftHand.launch`

(Most of the parameters of Adaptive Grasping can be changed from `adaptive_params.yaml` and `full_grasp_params.yaml`)

#### Launch Order

1. `roslaunch adaptive_grasping launchPandaSoftHand.launch` (Launches Robot in RViz + Real Robot)
2. `roslaunch panda_softhand_control launchControlServer.launch ` (Launches a server node which provides services for joint control, pose control, ecc.)
3. (IF NOT USING VISION) `roslaunch adaptive_grasping launchSimulateObjectPose.launch"` (check this file for the topic in which object pose should be published)
4. (IF NOT USING OBJECT VELOCITY ESTIMATION) `roslaunch adaptive_grasping launchSimulateObjectTwist.launch"` (check this file for the topic in which object twist should be published)
5. `roslaunch adaptive_grasping launchRobotCommAdaptiveGrasp.launch` (Launches adaptive grasping, full grasper and robotCommander)
6. `rosrun adaptive_grasping service_caller_node` (Performs the grasping by sequentially calling the needed services)
7. IF NOT USING THE IMU GLOVE, touch information should be publish on topic `/touching_finger_topic` as an `std_msgs/Int8`. For example, one can manually simulate touches by commanding `rostopic pub -r 50 /touching_finger_topic std_msgs/Int8 "data: 4"` (From 0 to 5 for fingers from thumb to pinky).

