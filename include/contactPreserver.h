#ifndef CONTACT_PRESERVER_H
#define CONTACT_PRESERVER_H

#include <kdl/jacobian.hpp>
#include <Eigen/Core>

/**
* @brief This class is called by the adaptive_grasping method to compute the contact preserving reference motions.
*
*/

class contactPreserver {

public:
  /**
  * @brief Default constructor for contactPreserver
  *
  * @param H
  *   the contact type of the robotic hand
  * @return null
  */
  contactPreserver();

  /**
  * @brief Default destructor for contactPreserver
  *
  * @param null
  * @return null
  */
  ~contactPreserver();

private:
  // Current hand jacobian
  KDL::Jacobian J;

  // Grasp Matrix
  Eigen::MatrixXd G;
}

#endif // CONTACT_PRESERVER_H
