//============================================================================
// Name        : renderspace.cpp
// Author      : Michael Andersch
// Version     :
// Copyright   : Michael Andersch, 2012
// Description : Main file of experiments with rendering approaches
// TODO List   : - Integrate DOF, distribution ray tracing
//				 - Build model loader and add triangle primitive
//               - Fix plane rendering with kd-tree
//				 - Integrate photon mapping + gridfile/kd-tree
//				 - Integrate video rendering functionality
//				 - Improve texture mapping system, i.e. do mipmaps
//============================================================================

#define EXIT_SUCCESS 0
#define EXIT_FAILURE -1

#include <iostream>
#include <cstdlib>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>

#include "Raytracer.hpp"
#include "SceneDescription.hpp"
#include "KdTree.hpp"

using namespace std;
using namespace cv;

typedef struct timeval timer;
#define TIME(x) gettimeofday(&x, NULL);

long timevaldiff(timer& start, timer& finish);

#define X_RESOLUTION 512
#define Y_RESOLUTION 512
#define DEFAULT_SCENE "simplescene"
#define RSPACE_VERSION_MAJOR 0
#define RSPACE_VERSION_MINOR 5

extern int totalIntersectionTests;

int main(int argc, char** argv) {
	cout << "RENDERSPACE Rendering Sandbox v" << RSPACE_VERSION_MAJOR << "." << RSPACE_VERSION_MINOR << endl;

	Mat outputImg;
	timer start, end;
	string sceneFileName = DEFAULT_SCENE;
	bool displayOnScreen = false;
	bool useAdaptiveAA = false;
	bool useBloom = false;
	unsigned xres = X_RESOLUTION, yres = Y_RESOLUTION;

	// copy scene file selection
	if(argc > 1)
		sceneFileName.assign(argv[1]);
	// enable rendered image display on screen if desired
	if(argc > 2)
		displayOnScreen = bool(!!atoi(argv[2]));
	// enable adaptive ssaa if desired
	if(argc > 3)
		useAdaptiveAA = bool(!!atoi(argv[3]));
	// allow specifying render resolution
	if(argc > 5) {
		xres = atoi(argv[4]);
		yres = atoi(argv[5]);
	}
	// bloom selector
	if(argc > 6)
		useBloom = bool(!!atoi(argv[6]));

	// read the scene description from disk
	SceneDescription* scene = new SceneDescription();
	if(!scene->readSceneFile(sceneFileName)) {
		std::cerr << "ERROR: Could not parse scene file!" << std::flush << std::endl;
		return EXIT_FAILURE;
	}
	scene->dumpScene();
	TIME(start); scene->buildKdTree(); TIME(end);
	double treeConstructionTime = (double)timevaldiff(start, end)/1000.0;
	scene->getKdTree()->print();

	// set up the rendering environment
	Renderer* renderer = new Raytracer();
	renderer->setRenderParams(xres, yres);
	renderer->configureAdaptiveSSAA(useAdaptiveAA);
	renderer->configureBloom(useBloom);
	renderer->setSceneDescription(scene);
	TIME(start); renderer->renderScene(); TIME(end);

	// print runtime stats
	cout << "time         : " << (double)timevaldiff(start, end)/1000.0 << "s" << endl;
	cout << "kdtree time  : " << treeConstructionTime << "s" << endl;
	cout << "intersections: " << totalIntersectionTests << endl;

	// do output
	outputImg = renderer->getRenderImage();
	if(displayOnScreen) {
		string outWinName("Rendering output");
		namedWindow(outWinName.c_str());
		imshow(outWinName.c_str(), outputImg);
		waitKey(0);
	} else {
		imwrite("result.png", outputImg);
	}

	return EXIT_SUCCESS;
}

/*
 *   Calculates the time difference between start and finish in msecs.
 */
long timevaldiff(timer& start, timer& finish) {
	long msec;
	msec = (finish.tv_sec - start.tv_sec) * 1000;
	msec += (finish.tv_usec - start.tv_usec) / 1000;
	return msec;
}
