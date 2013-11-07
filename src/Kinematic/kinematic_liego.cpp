#include "../include/Kinematic/kinematic_liego.h"
#include "../include/Core/Math/CMat44.h"
#include <memory>
#include <stdexcept>

using namespace handest;

/// A single instance of forward kinematics
ForwardKinematicsLiego::Ptr forward_kinematics;

Eigen::Matrix4f ForwardKinematicsLiego::matrixExp(Eigen::Matrix4f epsilon,
		double theta, int precision) {
	Eigen::Matrix4f result, cumulative;
	result.setIdentity();
	cumulative.setIdentity();

	for (int j = 0; j < precision; j++) {
		// (epsilon * theta) ^ j / j!
		cumulative = cumulative * epsilon * (theta / (j+1));

		// adding (epsilon * theta) ^ j  to result
		result = result + cumulative;
	}
	return result;
}

Mat34 ForwardKinematicsLiego::eigen2mat34(Eigen::Matrix4f trans) {
	Mat34 pose;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			pose.R.m[i][j] = trans(i, j);
		}
		pose.p.v[i] = trans(i, 3);
	}
	return pose;
}

Eigen::Matrix4f ForwardKinematicsLiego::getDash(Eigen::Vector3f vec) {
	Eigen::Matrix4f res;
	res.setZero();
	res(0, 0) = 0;
	res(0, 1) = -vec[2];
	res(0, 2) = vec[1];
	res(1, 0) = vec[2];
	res(1, 1) = 0;
	res(1, 2) = -vec[0];
	res(2, 0) = -vec[1];
	res(2, 1) = vec[0];
	res(2, 2) = 0;
	return res;
}

Eigen::Matrix4f ForwardKinematicsLiego::getEpsilon(Eigen::Vector3f omega,
		Eigen::Vector3f q) {
	Eigen::Matrix4f eps = getDash(omega);
	omega = -omega.cross(q);
	eps(0, 3) = omega[0];
	eps(1, 3) = omega[1];
	eps(2, 3) = omega[2];
	return eps;
}

void ForwardKinematicsLiego::fingerFK(Finger::Pose &finger, Finger::Config &config) {
	// Matrix with joints equal zero
	Eigen::Matrix4f zeroPos;
	zeroPos.setIdentity();

	// w_1 = (0, 1, 0) 		q_1 = (0,0,0);
	// w_2 = (1, 0, 0)		q_2 = (0,0,0);
	// w_3 = (0, 1, 0)		q_3 = (0,0,l1);
	// w_4 = (0, 1, 0)		q_4 = (0,0,l2);
	Eigen::Matrix4f epsX, epsY;
	epsY = getEpsilon(Eigen::Vector3f::UnitY(), Eigen::Vector3f(0, 0, 0));
	epsX = getEpsilon(Eigen::Vector3f::UnitX(), Eigen::Vector3f(0, 0, 0));

	// Chain 0
	finger.chain[0].pose = eigen2mat34(Eigen::Matrix4f::Identity());

	// Chain 1
	zeroPos(2, 3) = finger.chain[0].length;
	finger.chain[1].pose = eigen2mat34(
			matrixExp(epsY, config.conf[0]) * matrixExp(epsX, config.conf[1])
					* zeroPos);

	// Chain 2
	zeroPos(2, 3) = finger.chain[1].length;
	finger.chain[2].pose = eigen2mat34( matrixExp(epsY, config.conf[2]) * zeroPos);
}

void ForwardKinematicsLiego::forward(Hand::Pose &hand, Hand::Config &config)
{
	hand.palm.pose = eigen2mat34( Eigen::Matrix4f::Identity());

	for (int i = 0; i < Hand::FINGERS; i++) {
		Finger::Config fingerConfig;
		for (int j = 0; j < Finger::JOINTS; j++) {
			fingerConfig.conf[j] = config.conf[i * 5 + j];
		}

		fingerFK(hand.fingers[i], fingerConfig);
	}
}


handest::ForwardKinematics* handest::createForwardKinematicsLiego(void) {
	forward_kinematics.reset(new ForwardKinematicsLiego());
	return forward_kinematics.get();
}
