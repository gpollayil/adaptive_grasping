#ifndef CONTACT_PRESERVER_H
#define CONTACT_PRESERVER_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>

#include "task_utils/reversePriorityManager.h"
#include "task_utils/stackOfTasksManager.h"

/**
* @brief This class is called by the adaptive_grasping method to compute the
* contact preserving reference motions.
*
*/

namespace adaptive_grasping {

  class contactPreserver {

  public:

    /** DEFAULT CONSTRUCTOR
    * @brief Default constructor for contactPreserver
    *
    * @param null
    * @return null
    */
    contactPreserver();

    /** CONSTRUCTOR
    * @brief Overloaded constructor for contactPreserver
    *
    * @param S_
    *   the synergy matrix of the hand (if hand fully actuated S = I)
    * @return null
    */
    explicit contactPreserver(Eigen::MatrixXd S_);

    /** OTHER OVERLOADED CONSTRUCTOR
    * @brief Overloaded constructor for contactPreserver
    *
    * @param S_
    *   the synergy matrix of the hand (if hand fully actuated S = I)
    * @param num_tasks_
    *   the number of tasks for RP Mangager
    * @param dim_tasks_
    *   the dim of each task for RP Mangager
    * @param lamda_max_ and epsilon_
    *   the lamda_max_ and epsilon_ for RP Mangager
    * @return null
    */
    contactPreserver(Eigen::MatrixXd S_, int num_tasks_, std::vector<int> dim_tasks_, std::vector<int> prio_tasks_, double lambda_max_, double epsilon_);

    /** DESTRUCTOR
    * @brief Default destructor for contactPreserver
    *
    * @param null
    * @return null
    */
    ~contactPreserver();

    // A boolean for checking if the object has been initialized
    bool initialized = false;

    /** INITIALIZE
    * @brief Private function to initialize the object
    *
    * @param S_
    *   the synergy matrix of the hand (if hand fully actuated S = I)
    * @return null
    */
    bool initialize(Eigen::MatrixXd S_);

	/** INITIALIZETOPICS
    * @brief Private function to initialize the object
    *
    * @param object_twist_topic_name_
    *   the topic where the object twist is published
    * @return null
    */
	bool initialize_topics(std::string object_twist_topic_name_, ros::NodeHandle nh);

    /** INITIALIZETASKS
    * @brief Private function to initialize the object
    *
    * @param num_tasks_
    *   the number of tasks for RP Mangager
    * @param dim_tasks_
    *   the dim of each task for RP Mangager
    * @param lamda_max_ and epsilon_
    *   the lamda_max_ and epsilon_ for RP Mangager
    * @return null
    */
    bool initialize_tasks(int num_tasks_, std::vector<int> dim_tasks_, std::vector<int> prio_tasks_, double lambda_max_, double epsilon_);

    /** CHANGEHANDTYPE
    * @brief Function to eventually change the hand type (set new S)
    *
    * @param S_
    *   the new synergy matrix
    *
    * @return null
    */
    void changeHandType(Eigen::MatrixXd S_);

    /** SETGRASPSTATE
    * @brief Function to set new values for J, G, T and H
    *
    * @param J_
    *   the whole jacobian for the contacts
    * @param G_
    *   the whole grasp matrix for the contacts
    * @param T_
    *   the whole palm twist pole change matrix for the contacts
    * @param H_
    *   the new contact selection matrix
    * @return null
    */
    void setGraspState(Eigen::MatrixXd J_, Eigen::MatrixXd G_, Eigen::MatrixXd T_,
      Eigen::MatrixXd H_);

    /** SETMINIMIZATIONPARAMS
    * @brief Function to set new values for the minimization problem
    *
    * @param x_d_
    *   the desired motion of the hand given by some high level planner
    * @param f_d_d_
    *   the desired contact force derivative
    * @return null
    */
    void setMinimizationParams(Eigen::VectorXd x_d_, Eigen::VectorXd f_d_d_);

    /** SETPERMUTATIONPARAMS
    * @brief Function to set new permutation matrix for the relaxed minimization problem (used to compute R)
    *
    * @param P_
    *   the whole permutation matrix
    * @param num_contacts_
    *   the number of fingers in contact
    * @return null
    */
    void setPermutationParams(Eigen::MatrixXd P_, int num_contacts_);

    /** PERFORMKININVERSTION
    * @brief Function to perform a simple task inversion(num_tasks and dim_tasks)
    *
    * @param x_result
    *   the resulting motion that preserves contacts
    * @return bool success if the found result is a valid one
    */
    bool performKinInversion(Eigen::VectorXd& x_result);

    /** PRINTALL
    * @brief Function to print out to console all relevant variables
    *
    * @return null
    */
    void printAll();

  private:
    // Boolean for checking if first iteration
    bool first_it = true;

    // Current contacts jacobian
    Eigen::MatrixXd J;

    // Grasp matrix
    Eigen::MatrixXd G;

    // Pole change matrix (twist of palm to contacts)
    Eigen::MatrixXd T;

    // Contact selection Matrix
    Eigen::MatrixXd H;

    // Synergy Matrix
    Eigen::MatrixXd S;

    // Permutation Matrix
    Eigen::MatrixXd P;

    // Planner desired motions
    Eigen::VectorXd x_d;

    // Previous planner desired motions
    Eigen::VectorXd x_d_old;

    // The object twist set by the object twist subscriber
    Eigen::VectorXd xi_o;

    // Planner desired contact forces derivative
    Eigen::VectorXd f_d_d;

    // Full minimization solution
    Eigen::VectorXd x_ref;

    // Full minimization solution
    Eigen::VectorXd x_ref_old;

    // Constant term for Q_tilde (obtained by appending zeros under x_d)
    Eigen::VectorXd y;

    // Contact relation matrix
    Eigen::MatrixXd Q;

    // Old contact relation matrix
    Eigen::MatrixXd Q_old;

    // Q_tilde matrix
    Eigen::MatrixXd Q_tilde;

    // Relaxation matrix R
    Eigen::MatrixXd R;

    // Relaxation matrix R_bar
    Eigen::MatrixXd R_bar;

    // Pseudo inverse of R_bar Q_tilde
    Eigen::MatrixXd pinv_R_bar_Q_tilde;

    // The C matrix in the algorithm
    Eigen::MatrixXd C;

    // Object twist topic, subscriber and needed node handle.
    std::string object_twist_topic_name;
    std::unique_ptr<ros::NodeHandle> cp_nh_ptr;
    ros::Subscriber obj_twist_sub;

    // Number of times the relax functon has been called for the same x_d
    int relaxation_order;

    // Number of contacting fingers
    int num_contacts;

    // Number of tasks for RP Manager
    int num_tasks;

    // The dimensions of each task for RP Manager (Obviously it must have same no of elements as num_tasks)
    std::vector<int> dim_tasks;

    // The priority of the above defined tasks
    std::vector<int> prio_tasks;

    // The lambda_max for RP Manager
    double lambda_max;

    // The epsilon for RP Manager
    double epsilon;

    // Inverse Kinematics Managers
    reversePriorityManager rp_manager;
    stackOfTasksManager sot_manager;

    // Temporary task for filling up the task vec
    basicTask tmp_task;

    // Temporary task vector for filling up the rp_manager
    std::vector<basicTask> tmp_task_vec;

    // Null space basis of Q_tilde
    Eigen::MatrixXd N_tilde;

    // PRIVATE FUNCTIONS

	/** OBJECTTWISTCALLBACK
    * @brief Callback function to get the object twist from a topic
    *
    * @param msg
    * @return null
    */
	void object_twist_callback(const geometry_msgs::Twist::ConstPtr &msg);

  }; // closing class

} // closing namespace



#endif // CONTACT_PRESERVER_H
