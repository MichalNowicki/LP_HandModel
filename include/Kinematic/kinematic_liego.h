#ifndef _fk
#define _fk

#include "../handest_defs.h"
#include "../../dependencies/Eigen/Eigen"
#include "forward_kinematics.h"
#include <iostream>
#include <memory>

namespace handest {
	/// create a single forward kinematic
	ForwardKinematics* createForwardKinematicsLiego(void);
};


using namespace handest;

class ForwardKinematicsLiego : public ForwardKinematics {

private:
	/// Returns the ^ on vec = (vec1, vec2, vec3)
	/// vec = [ 	0, 	-vec3,	 vec2;
	///		   vec3, 	0, 	-vec1;
	///		  -vec2, vec1, 		0 ]
	Eigen::Matrix4f getDash(Eigen::Vector3f vec);

	/// Calculates the eps matrix:
	/// eps = [ omega^ 	, - omega x q;
	///			0 		, 		0 		]
	Eigen::Matrix4f getEpsilon(Eigen::Vector3f omega, Eigen::Vector3f q);

	/// Implements the matrix exponential based on epsilon and theta:
	/// e ^ (eps*theta) = I + eps*theta + eps*theta/2! + ...
	/// precision informs about the number of "eps*theta" elements that are used
	Eigen::Matrix4f matrixExp(Eigen::Matrix4f epsilon, double theta,
			int precision = 10);

	/// Conversion from eigen matrices to mat34
	Mat34 eigen2mat34(Eigen::Matrix4f trans);

	/// Recalculates "pose" matrices based on the values of joints in config
	void fingerFK(Finger::Pose &finger, Finger::Config &config);

public:
	/// Pointer
	typedef std::unique_ptr<ForwardKinematics> Ptr;

	/// Calculates pose matrices based on given config
	void forward(Hand::Pose& pose, Hand::Config &config);
};

#endif
