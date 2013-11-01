#include "../include/Kinematic/kinematic.h"
#include "../include/Core/Math/CMat44.h"

Eigen::Matrix4f ForwardKinematics::matrixExp(Eigen::Matrix4f epsilon, double theta, int precision)
{
	Eigen::Matrix4f result, cumulative;
	result.setIdentity();
	cumulative.setIdentity();

	float factorial = 1;
	for (int j = 0; j < precision; j++)
	{
		// (epsilon * theta) ^ j
		cumulative = cumulative* epsilon* (theta / factorial);

		// Next iteration is (j+2)!
		factorial *= (j+2);

		// adding (epsilon * theta) ^ j  to result
		result = result + cumulative;
	}
	return result;
}

Mat34 ForwardKinematics::eigen_2_mat34(Eigen::Matrix4f trans)
{
	Mat34 pose;
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			pose.R.m[i][j] = trans(i,j);
		}
		pose.p.v[i] = trans(i,3);
	}
	return pose;
}

Eigen::Matrix4f ForwardKinematics::mat34_2_eigen(Mat34 trans)
{
	Eigen::Matrix4f pose;
	pose.setIdentity();
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			pose(i,j) = trans.R.m[i][j];
		}
		pose(i,3) = trans.p.v[i];
	}
	return pose;
}


Eigen::Matrix4f ForwardKinematics::getDash(Eigen::Vector3f vec)
{
	Eigen::Matrix4f res;
	res.setZero();
	res(0,0) = 0;
	res(0,1) = - vec[2];
	res(0,2) = vec[1];
	res(1,0) = vec[2];
	res(1,1) = 0;
	res(1,2) = - vec[0];
	res(2,0) = - vec[1];
	res(2,1) = vec[0];
	res(2,2) = 0;
	return res;
}

Eigen::Matrix4f ForwardKinematics::getEpsilon(Eigen::Vector3f omega, Eigen::Vector3f q)
{
	Eigen::Matrix4f eps = getDash( omega );
	omega = - omega.cross( q );
	eps(0,3) = omega[0];
	eps(1,3) = omega[1];
	eps(2,3) = omega[2];
	return eps;
}

void ForwardKinematics::fingerFK(Finger::Pose finger, Finger::Config config, float *length)
{
	// Matrix with joints equal zero
	Eigen::Matrix4f zeroPos;
	zeroPos.setIdentity();
	zeroPos(2,3) = length[0] + length[1] + length[2];

	// first joint
	// w_1 = (0, 1, 0) 		q_1 = (0,0,0);
	// w_2 = (1, 0, 0)		q_2 = (0,0,0);
	// w_3 = (0, 1, 0)		q_3 = (0,0,l1);
	// w_4 = (0, 1, 0)		q_4 = (0,0,l1+l2);
	Eigen::Matrix4f eps[4];
	eps[0] = getEpsilon(Eigen::Vector3f::UnitY() , Eigen::Vector3f(0,0,0) );
	eps[1] = getEpsilon(Eigen::Vector3f::UnitX() , Eigen::Vector3f(0,0,0) );
	eps[2] = getEpsilon(Eigen::Vector3f::UnitY() , Eigen::Vector3f(0,0,length[0]) );
	eps[3] = getEpsilon(Eigen::Vector3f::UnitY() , Eigen::Vector3f(0,0,length[0] + length[1]) );

	for (int i=0;i<Finger::LINKS;i++)
	{
		Eigen::Matrix4f trans = zeroPos * mat34_2_eigen(finger.pose);
		for (int j=i+1;j>=0;j--)
		{
			trans = matrixExp(eps[j], config.conf[j]) * trans;
		}
		finger.chain[i].pose = eigen_2_mat34(trans);
	}
}

void ForwardKinematics::handFK(Hand::Pose hand, Hand::Config config)
{
	/// temporal assumption
	float length[3] = {1.0, 1.0, 1.0};

	for (int i=0; i< Hand::FINGERS; i++)
	{
		Finger::Config fingerConfig;
		for (int j=0; j< Finger::JOINTS; j++)
		{
			fingerConfig.conf[j] = config.conf[i*5 + j];
		}

		fingerFK( hand.fingers[i], fingerConfig, length ) ;
	}
}
