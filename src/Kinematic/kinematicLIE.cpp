#include "../include/Kinematic/kinematic.h"

CMat44 ForwardKinematics::matrixExp(CMat44 epsilon, float_t theta, int precision)
{
	CMat44 result, cumulative;
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


void ForwardKinematics::fingerFK(Finger::Pose finger, Finger::Config config)
{
	float l1 = 1, l2 = 1, l3 = 1;
	CMat44 zeroPos;
	zeroPos.setIdentity();
	zeroPos.pos[2] = l1 + l2 + l3;

	CMat44 eps;
	for (int i=0;i<Finger::LINKS;i++)
	{
		if ( i == 0)
		{

			finger.chain[i].pose = matrixExp(eps, config.conf[0]);
		}
	}
}

void ForwardKinematics::handFK(Hand::Pose hand, Hand::Config config)
{
	for (int i=0; i< Hand::FINGERS; i++)
	{
		Finger::Config fingerConfig;
		for (int j=0; j< Finger::JOINTS; j++)
		{
			fingerConfig.conf[j] = config.conf[i*5 + j];
		}

		fingerFK( hand.fingers[i], fingerConfig ) ;
	}


}
