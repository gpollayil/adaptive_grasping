//
// Created by George on 20/11/19.
//

#ifndef SRC_INVERSION_UTILITIES_H
#define SRC_INVERSION_UTILITIES_H

#include <Eigen/Dense>
#include "ros/ros.h"

/**
* @brief This h file contains utilities for pseudo inversions
**/

inline Eigen::MatrixXd damped_pseudo_inv(Eigen::MatrixXd input_mat, double damping_coeff, double epsilon) {
	// Computing the singular values
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();

	// Checking if the smallest sing val is really small
	double lambda_sq = 0.0;
	double min_sing_val = sing_vals(sing_vals.size() - 1);
	if (min_sing_val < epsilon) {
		lambda_sq = (1 - pow((min_sing_val / epsilon), 2)) * pow(damping_coeff, 2);
		ROS_WARN("Damping the pseudo inverse!!!");
	}

	// Changing the diagonal sv matrix
	Eigen::MatrixXd S = input_mat; S.setZero();
	for (int i = 0; i < sing_vals.size(); i++) {
		S(i, i) = (sing_vals(i)) / (pow(sing_vals(i), 2) + lambda_sq);
	}

	// Return the svd based damped pseudoinverse
	return Eigen::MatrixXd(svd.matrixV() * S.transpose() * svd.matrixU().transpose());
}

inline Eigen::MatrixXd trunk_pseudo_inv(Eigen::MatrixXd input_mat, double epsilon) {
	// Computing the singular values
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();

	// Checking if the smallest sing val is really small
	bool really_small = false;
	double min_sing_val = sing_vals(sing_vals.size() - 1);
	if (min_sing_val < epsilon) {
		really_small = true;
		ROS_WARN("The min sing val is really small! Will set it to zero for the pseudo inverse!!!");
	}

	// Changing the min sing value in the sv matrix
	Eigen::MatrixXd S = input_mat;
	if (really_small) S(sing_vals.size() - 1, sing_vals.size() - 1) = 0.0;

	// Return the svd based damped pseudoinverse
	return Eigen::MatrixXd(svd.matrixV() * S.transpose() * svd.matrixU().transpose());
}

#endif //SRC_INVERSION_UTILITIES_H
