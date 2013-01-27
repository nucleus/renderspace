/*
 * Camera.hpp
 *
 *  Created on: 28.12.2012
 *      Author: michael
 */

#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "glm.hpp"
#include "Ray.hpp"
#include <cmath>

/*!
 * A camera class that constructs appropriate rays from given camera parameters.
 */
class Camera {
public:
	Camera(glm::vec3 pos, glm::vec3 lookat, glm::vec3 up, float fovx) {
		// for some reason, the y axis seems to be inverted - love trial&error ;)
		this->pos = pos;
		this->dir = glm::normalize(lookat-this->pos);
		this->right = glm::normalize(glm::cross(this->dir, up));
		this->up = glm::normalize(glm::cross(this->right, this->dir));
		this->fovx = fovx * M_PI / 180.0;
		this->dist = 0.5 * tan(this->fovx/2.0);
	}

	/*
	 * Function: constructRayForPixel
	 *
	 * Creates primary camera rays for a given pixel coordinate (x,y) and image resolution
	 * (xres, yres).
	 */
	Ray constructRayForPixel(float x, float y, unsigned xres, unsigned yres) {
		Ray ray(pos, dist * dir +
                (y / (float) (yres - 1) - 0.5) * up +
                (x / (float) (xres - 1) - 0.5) * right);
		return ray;
	}

	glm::vec3& getPos() { return pos; }
	glm::vec3& getDir() { return dir; }
	glm::vec3& getUp()  { return up; }
private:
	glm::vec3 pos, dir, up, right;
	float dist, fovx;
};

#endif /* CAMERA_HPP_ */
