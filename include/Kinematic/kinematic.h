#ifndef _fk
#define _fk

#include "../handest_defs.h"
#include "../../dependencies/Eigen/Eigen"

using namespace handest;

class ForwardKinematics {
//public:
	// Implements the matrix exponential based on epsilon and theta:
	// e ^ (eps*theta) = I + eps*theta + eps*theta/2! + ...
	// precision informs about the number of "eps*theta" elements that are used in calculation before
	// cutting the rest out
	Eigen::Matrix4f matrixExp(Eigen::Matrix4f epsilon, double theta,
			int precision = 10);

	// Recalulcation from eigen Matrix to mat34 and from mat34 to eigen
	Mat34 eigen_2_mat34(Eigen::Matrix4f trans);
	Eigen::Matrix4f mat34_2_eigen(Mat34 trans);

	// Returns the vec^
	Eigen::Matrix4f getDash(Eigen::Vector3f vec);

	// Calculates the eps matrix:
	// eps = [ omega^ 	, - omega x q;
	//			0 		, 		0 		]
	Eigen::Matrix4f getEpsilon(Eigen::Vector3f omega, Eigen::Vector3f q);

public:
	/// Recalculates "pose" matrices based on the values of joints in config
	void fingerFK(Finger::Pose *finger, Finger::Config config, float *lengths);

	/// Recalculates "pose" matrices of whole hand based on the config
	void handFK(Hand::Pose *hand, Hand::Config config);


	// Forward(Hand::Pose& pose, Hand::Config &config);
};

#endif
