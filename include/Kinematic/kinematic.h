#ifndef _fk
#define _fk

#include "../handest_defs.h"

using namespace handest;

class ForwardKinematics
{
	CMat44 matrixExp(CMat44 epsilon, float_t theta, int precision = 5);
	void fingerFK(Finger::Pose finger, Finger::Config config);
	void handFK(Hand::Pose hand, Hand::Config config);
};

#endif
