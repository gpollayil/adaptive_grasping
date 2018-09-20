#include "matricesCreator.h"

/**
* @brief The following are functions of the class contactPreserver.
*
*/

using namespace adaptive_grasping;

/* CONSTRUCTOR */
matricesCreator::matricesCreator(Eigen::MatrixXd H_i_, std::string world_frame_name_,
  std::string palm_frame_name_){
    // Set the basic contact selection matrix
    changeHandType(H_i_);

    // Set the frame names for world and palm
    changeFrameNames(world_frame_name_, palm_frame_name_);
}

/* DESTRUCTOR */
matricesCreator::~matricesCreator(){
  // Do nothing
}

/* CHANGEHANDTYPE */
void matricesCreator::changeHandType(Eigen::MatrixXd H_i_){
  // Set the new basic contact selection matrix
  H_i = H_i_;
}

/* CHANGEFRAMENAMES */
void matricesCreator::changeFrameNames(std::string world_frame_name_,
  std::string palm_frame_name_){
    // Set the new frame names for world and palm
    world_frame_name = world_frame_name_; palm_frame_name = palm_frame_name_;
}

/* SETCONTACTSMAP */
void matricesCreator::setContactsMap(std::map<int, std::tuple<std::string,
  Eigen::Affine3d, Eigen::Affine3d>> contacts_map_){
    contacts_map = contacts_map_;
}

/* SETOBJECTPOSE */
void matricesCreator::setObjectPose(Eigen::Affine3d object_pose_){
  object_pose = object_pose_;
}
