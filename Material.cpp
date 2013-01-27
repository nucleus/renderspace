/*
 * Material.cpp
 *
 *  Created on: 10.01.2013
 *      Author: michael
 */

#include "Material.hpp"

/* ideally diffuse material */
const Material idealDiffMaterial(	glm::vec3(0.3f),
									glm::vec3(0.7f),
									glm::vec3(0.7f),
									16.0, 0.0, 0.0, 1.0 );
#if 1
/* ideally reflective material */
const Material idealReflMaterial(	glm::vec3(0.2f),
									glm::vec3(0.8f),
									glm::vec3(2.0f),
									16.0, 1.0, 0.0, 0.6 );
#else
/* ideally reflective material */
const Material idealReflMaterial(	glm::vec3(0.3f),
									glm::vec3(0.3f, 0.05f, 0.05f),
									glm::vec3(0.1f, 0.9f, 0.9f),
									128.0f, 0.0f, 0.0f, 1.0f );
#endif

/* ideally refractive material */
const Material idealRefrMaterial(	glm::vec3(0.0f),
									glm::vec3(0.6f),
									glm::vec3(2.0f),
									16.0, 0.1, 1.0, 0.0 );

std::list<glm::vec3>* AreaLight::getSamplingPositions() {
	std::list<glm::vec3>* samples = new std::list<glm::vec3>();

	glm::vec3 base = pos;
	base += 0.5f * step;
	glm::vec3 sample;

	for(unsigned i = 0; i < numSamples1D; i++) {
		for(unsigned j = 0; j < numSamples1D; j++) {
			sample = base + (drand48()-0.5) * step;
			samples->push_back(sample);
			base[dim0] += step[dim0];
		}
		base[dim0] = pos[dim0] + 0.5 * step[dim0];
		base[dim1] += step[dim1];
	}

	return samples;
}
