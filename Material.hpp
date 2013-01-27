/*
 * Material.hpp
 *
 *  Created on: 24.12.2012
 *      Author: michael
 */

#ifndef MATERIAL_HPP_
#define MATERIAL_HPP_

#include "glm.hpp"
#include "common.hpp"
#include <iostream>
#include <list>
#include <cstdlib> // for drand48
#include <ctime>   // for clock

// for the moment, the number of samples distributed over an area light is fixed
#define AL_SAMPLES 8

/*
 * Can have point and area lights
 */
enum LightType {
	LT_POINT = 1,
	LT_AREA = 2
};

/*
 * General light source class. Specifies only the color of the light.
 */
class Lightsource {
public:
	glm::vec3 color;	// color of the light source

	LightType type;		// holds light type info for derived classes

	virtual void print() = 0;
};

/*
 * A point light which has infinitely small extents and a single location in space.
 */
class PointLight : public Lightsource {
public:
	PointLight(glm::vec3 pos, glm::vec3 color) {
		this->pos = pos;
		this->color = color;
		this->type = LT_POINT;
	}

	glm::vec3 pos;	// light position in 3d space

	virtual void print() {
		std::cout << "Type POINT: at (" << pos.x << "," << pos.y << "," << pos.z << "), color (";
		std::cout << color.r << "," << color.g << ","	<< color.b << ")" << std::endl;
	}
};

/*
 * An axis-aligned area light source for producing soft shadows with distribution ray tracing.
 */
class AreaLight : public Lightsource {
public:
	AreaLight(glm::vec3 pos, glm::vec3 extents, glm::vec3 color, unsigned numSamples1D) {
		this->pos = pos;
		this->extents = extents;
		this->color = color;
		this->numSamples1D = numSamples1D;
		this->type = LT_AREA;

		// confirm that area light is really flat
		if(extents.x == 0.0) {
			dim0 = 1; dim1 = 2;
		} else if(extents.y == 0.0) {
			dim0 = 0; dim1 = 2;
		} else if(extents.z == 0.0) {
			dim0 = 0; dim1 = 1;
		} else {
			std::cerr << "ERROR: Area light can extend in only two dimensions!" << std::flush << std::endl; exit(EXIT_FAILURE);
		}

		// calculate steps between cells on the area light
		step = extents * (1.0 / (double)numSamples1D);

		// seed the random number generator for distribution ray tracing
		srand48((long int)clock());
	}

	glm::vec3 pos, extents, step;
	unsigned numSamples1D;
	int dim0, dim1;

	virtual void print() {
		std::cout << "Type AREA: at (" << pos.x << "," << pos.y << "," << pos.z << "), extents (";
		std::cout << extents.x << "," << extents.y << "," << extents.z << "), color (";
		std::cout << color.r << "," << color.g << ","	<< color.b << ")" << std::endl;
	}

	// key function, returns randomly distributed samples over the surface of the light
	std::list<glm::vec3>* getSamplingPositions();
};

/*
 * Material class. Contains ambient, diffuse, and specular multipliers for object color,
 * and scaling factors for the contributions of self-color, reflected color, and refracted
 * color. Still experimenting with material parameters and how to model materials the best way.
 */
class Material {
public:
	Material() {
		ambient = glm::vec3(0.0f);
		diffuse = glm::vec3(0.0f);
		specular = glm::vec3(0.0f);
		shininess = 0.0f;
		selfLightContrib = reflLightContrib = 0.33;
		refrLightContrib = 0.34;
		refractionIndex = 1.3;
	}

	Material(glm::vec3 amb, glm::vec3 diff, glm::vec3 spec, float exp, float refll, float refrl, float diffl) {
		ambient = amb;
		diffuse = diff;
		specular = spec;
		shininess = exp;

		// catch bad values
		// TODO: should support true exceptions in the future
		if(refll > 1.0)
			refll = 1.0;
		if(refrl > 1.0)
			refrl = 1.0;
		if(diffl > 1.0)
			diffl = 1.0;

		selfLightContrib = diffl;
		reflLightContrib = refll;
		refrLightContrib = refrl;

		refractionIndex = 1.2;
	}

	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
	float shininess;

	float refractionIndex;

	float selfLightContrib;
	float reflLightContrib;
	float refrLightContrib;
};

extern const Material idealDiffMaterial;
extern const Material idealReflMaterial;
extern const Material idealRefrMaterial;

#endif /* MATERIAL_HPP_ */
