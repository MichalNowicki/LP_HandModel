#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Core/Math/CMat44.h"
#include "Kinematic/kinematic.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

int main()
{
    try {
        using namespace handest;

        Grabber* grabber = createGrabberKinect();
        cout << "Current grabber: " << grabber->getName() << endl;
        Point3D::Cloud scene;
        grabber->grab();
        grabber->getCloud(scene);
        //Visualizer* visualizer = createVisualizerGL();
        //visualizer->showCloud(scene);
        //Filter* filter = createFilterPCL();
        Point3D::Cloud hand_cloud;
        //filter->FilterScene(scene, hand_cloud);

        Hand::Pose hand;
        //OptimizationFunction * optimization_function = createOptimizationGauss();
        //Optimization * optimization = createOptimizationPSO();
        //optimization->optimize(hand_cloud, hand);
        //optimization->save2File(hand);
        //visualizer->showHand(hand);

      //  CMat44 matrix1;
      //  matrix1.createTRMatrix(0, M_PI/2, 0, 0.1, 0.2, 0.3);
      ///  matrix1.showMatrix();
       // matrix1.inv(&matrix1);
      //  matrix1.showMatrix();
    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}


	/// Testing scenarios for FK
	Finger::Pose *finger;
	finger = new Finger::Pose();
	Mat33 z;
	z.m[0][0] = z.m[1][1] = z.m[2][2] = 1.0;
	finger->pose.R = z;
	Vec3 g;
	finger->pose.p = g;
	finger->pose.p.v[2] = 0.0;//10.0;

	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			printf("%f ", finger->pose.R.m[i][j]);
		}
		printf("%f ", finger->pose.p.v[i]);
		printf("\n");
	}

	ForwardKinematics *fk;
	fk = new ForwardKinematics();

	Finger::Config config;
	config.conf[0] = 0;
	config.conf[1] = 0;
	config.conf[2] = -45 * 3.14/180;
	config.conf[3] = 90 * 3.14/180;
	float len[3] = {1, 2, 2 };
	fk->fingerFK(finger,config,len);

	for (int i=0;i<3;i++)
	{
		printf("%f %f %f \n", finger->chain[i].pose.p.v[0], finger->chain[i].pose.p.v[1], finger->chain[i].pose.p.v[2]);
	}

	//std::cout<<std::endl<<fk->getEpsilon(Eigen::Vector3f::UnitZ(), Eigen::Vector3f(0, 0, 0))<<std::endl;

	//Eigen::Matrix4f x = fk->matrixExp(fk->getEpsilon(Eigen::Vector3f::UnitZ(), Eigen::Vector3f(0, 0, 0)), config.conf[0], 15);

	//std::cout<<std::endl<<x<<std::endl;

	return 0;
}
