#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"

using namespace std;

int main()
{
    try {
        using namespace handest;

        Grabber* grabber = createGrabberKinect();
        cout << "Current grabber: " << grabber->getName();
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
    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}
    return 0;
}
