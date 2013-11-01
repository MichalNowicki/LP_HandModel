#ifndef _fk
#define _fk

#include "../handest_defs.h"

using namespace handest;

class ForwardKinematics
{
	void fingerFK(Finger::Pose finger, Finger::Config config);
	void handFK(Hand::Pose hand, Hand::Config config);
};

#endif
