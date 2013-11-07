#ifndef _FK_H_
#define _FK_H_

#include "../handest_defs.h"


namespace handest {

	/// Forward Kinematic interface
	class ForwardKinematics {
	public:
		// Calculates Hand forward kinematic according to the config file
		virtual void forward(Hand::Pose& pose, Hand::Config &config) = 0;

		/// Virtual descrutor
		virtual ~ForwardKinematics() {}
	};
};

#endif
