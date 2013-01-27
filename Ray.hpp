/*
 * Ray.hpp
 *
 *  Created on: 25.12.2012
 *      Author: michael
 */

#ifndef RAY_HPP_
#define RAY_HPP_

#include <iostream>

#include "glm.hpp"
#include "SceneDescription.hpp"
#include "common.hpp"

extern long long rayIDGenerator;

/*
 * Class: Ray
 *
 * The core class of the raytracer. Represents a single ray in 3D space.
 */
class Ray {
public:
	Ray(glm::vec3 org, glm::vec3 dir) {
		this->org = org;
		this->dir = glm::normalize(dir);
		this->invdir = glm::normalize(glm::vec3(1.0/dir.x, 1.0/dir.y, 1.0/dir.z));
		for(int i = 0; i < 3; i++) if(dir[i] > 0.001f || dir[i] < -0.001f) {
			nzIndex = i;
			break;
		}

		rayID = __sync_fetch_and_add(&rayIDGenerator, 1);
	}

	/* Ray dim3 origin and dim3 direction */
	glm::vec3 org, dir, invdir;

	char nzIndex;

	long long rayID;

	/*
	 * Function: getTValueForRayPoint
	 *
	 * Returns the ray marching parameter t for a given point on the ray.
	 * Since t is constant for all three components, must compute only one.
	 */
	float getTValueForRayPoint(glm::vec3 rp) {
		return ((rp[nzIndex]-org[nzIndex]) / dir[nzIndex]);
	}

	/*
	 * Function: print
	 *
	 * Prints the ray's direction and origin vectors.
	 */
	void print(std::ostream& stream) {
		stream << "Ray (" << rayID << "): (" << org.x << "," << org.y << "," << org.z << ") + t * (";
		stream << dir.x << "," << dir.y << "," << dir.z << ")" << std::endl;
	}
};

#endif /* RAY_HPP_ */
