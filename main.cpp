#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Core/Math/CMat44.h"
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

        CMat44 matrix1;
        matrix1.createTRMatrix(0, M_PI/2, 0, 0.1, 0.2, 0.3);
        matrix1.showMatrix();
        matrix1.inv(&matrix1);
        matrix1.showMatrix();
    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}
    return 0;
}
