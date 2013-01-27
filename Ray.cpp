/*
 * Ray.cpp
 *
 *  Created on: 21.01.2013
 *      Author: michael
 */

#include "Ray.hpp"

static unsigned long long rayIDGenerator = 0;

Ray::Ray(glm::vec3 org, glm::vec3 dir) {
	this->org = org;
	this->dir = glm::normalize(dir);
	this->invdir = glm::normalize(glm::vec3(1.0/dir.x, 1.0/dir.y, 1.0/dir.z));

	for(int i = 0; i < 3; i++) if(dir[i] > 0.001f || dir[i] < -0.001f) {
		nzIndex = i;
		break;
	}

	rayID = __sync_fetch_and_add(&rayIDGenerator, 1);
}


float Ray::getTValueForRayPoint(glm::vec3 rp) {
	return ((rp[nzIndex]-org[nzIndex]) / dir[nzIndex]);
}

void Ray::print(std::ostream& stream) {
	stream << "Ray: (" << org.x << "," << org.y << "," << org.z << ") + t * (";
	stream << dir.x << "," << dir.y << "," << dir.z << ")" << std::endl;
}
