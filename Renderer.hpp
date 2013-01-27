/*
 * Renderer.hpp
 *
 *  Created on: 23.12.2012
 *      Author: michael
 */

#ifndef RENDERER_HPP_
#define RENDERER_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include "SceneDescription.hpp"

class Renderer {

public:
	Renderer() { scene = NULL; useAdaptiveSSAA = false; useBloom = false; }
	~Renderer() { if(scene) delete scene; }

	/*
	 * Function: renderScene
	 *
	 * This is the core function of the renderer object. It will render
	 * an image of the currently set scene with the given rendering
	 * parameters.
	 */
	virtual bool renderScene() = 0;

	void setSceneDescription(SceneDescription* scene) { this->scene = scene; }
	void setRenderParams(unsigned xres, unsigned yres) {
		this->xres = xres;
		this->yres = yres;
	}
	void configureAdaptiveSSAA(bool AASwitch) { useAdaptiveSSAA = AASwitch; }
	void configureBloom(bool doBloom) { useBloom = doBloom; }
	Mat& getRenderImage() { return this->outImage; }

protected:
	SceneDescription* scene;
	Mat outImage;

	unsigned int xres, yres;
	bool useAdaptiveSSAA;
	bool useBloom;
};

#endif /* RENDERER_HPP_ */
