#include "../include/Kinematic/kinematic.h"

void ForwardKinematics::fingerFK(Finger::Pose finger, Finger::Config config)
{


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
