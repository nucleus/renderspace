/*
 * Raytracer.cpp
 *
 *  Created on: 23.12.2012
 *      Author: michael
 */

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <omp.h>

#include "Raytracer.hpp"
#include "Camera.hpp"
#include "gtx/fast_square_root.hpp"

long long rayIDGenerator = 0;

/*
 * Int-to-string conversion.
 */
static std::string convertInt(int number) {
   std::stringstream ss; //create a stringstream
   ss << number; //add number to the stream
   return ss.str(); //return a string with the contents of the stream
}

glm::vec3 Raytracer::refraction(Ray& ray, glm::vec3& hitPoint, Object* obj, int remainBounces) {
	float n = 1.0f/obj->material.refractionIndex;
	float tIntersect;

	// get the refracted ray through the object
	glm::vec3 objNormal = obj->computeNormal(hitPoint);
	Ray refractionRay(hitPoint-0.01*objNormal, glm::refract(ray.dir, objNormal, n));
	glm::vec3 color(0.0f);

	// get the refracted ray at the other side of the object
	if(!obj->intersect(refractionRay, tIntersect)) return glm::vec3(0.0);
	glm::vec3 insideHit = refractionRay.org + tIntersect * refractionRay.dir;
	glm::vec3 objNormalRefr = obj->computeNormal(insideHit);
	refractionRay.org = insideHit + 0.01*objNormalRefr;
	refractionRay.dir = glm::refract(refractionRay.dir, -objNormalRefr, 1.0/n);
	return trace(refractionRay, NULL, remainBounces-1, obj->id);
}

bool Raytracer::findNearest(Ray& ray, glm::vec3& hit, Object** closestObj, unsigned long long excludeID, float maxDist) {
	float curDist;
	bool intersected = false;
	glm::vec3 dummy;
	float tIntersect;

	std::list<Object*>::iterator it;
	std::list<Object*>& objs = scene->getObjects();
	for(it = objs.begin(); it != objs.end(); ++it) {
		if((*it)->id != excludeID && (*it)->intersect(ray, tIntersect)) {
			dummy = ray.org + tIntersect * ray.dir;
			if((curDist = dist(ray.org, dummy)) < maxDist) {
				intersected = true;
				maxDist = curDist;
				hit = dummy;
				*closestObj = *it;
			}
		}
	}
	return intersected;
}

bool Raytracer::findLightOccluder(glm::vec3& start, glm::vec3& target, unsigned long long excludeID) {
#if USE_KDTREE == 1
	return scene->getKdTree()->traverseAndFindLightOccluders(start, target);
#else
	Ray lightRay(start, glm::normalize(target-start));
	float distance = dist(start, target);
	float tIntersect;

	std::list<Object*>::iterator objIt;
	std::list<Object*>& objects = scene->getObjects();

	for(objIt=objects.begin(); objIt != objects.end(); ++objIt) {
		if(((*objIt)->id != excludeID) && (*objIt)->intersect(lightRay, tIntersect) && (*objIt)->material.refrLightContrib == 0.0) {
			// a shadow is cast only if the found intersection is between object and light source
			if(dist(start, lightRay.org + tIntersect * lightRay.dir) < distance)
				return false;
		}
	}
	return true;
#endif
}

glm::vec3 Raytracer::trace(Ray& ray, Object** primaryObj, int remainBounces, unsigned long long sourceObjID) {
	// break recursion if desired depth reached
	if(remainBounces == 0)
		return glm::vec3(0.0f);

	// accumulate color in this value
	glm::vec3 color(0.0f);

	// intersection information
#if USE_KDTREE == 1
	float tIntersect;
#endif
	glm::vec3 closestIntersection;
	Object* closestObj = NULL;

	// find closest object intersection if it exists
#if USE_KDTREE == 1
	bool intersectedObjects = scene->getKdTree()->traverseAndIntersect(ray, closestObj, tIntersect, sourceObjID);
	closestIntersection = ray.org + tIntersect * ray.dir;
#else
	bool intersectedObjects = findNearest(ray, closestIntersection, &closestObj, sourceObjID, (float)1e10);
#endif
	// if the ray intersected any objects, calculate color
	if(intersectedObjects) {
		// store primary object for adaptive supersampling
		if(useAdaptiveSSAA && (int)sourceObjID == PRIMARY_RAY_OBJID)
			*primaryObj = closestObj;

		// accumulate lighting from all light sources
		float s = closestObj->material.selfLightContrib;
		if(s > 0.0) {
			std::list<Lightsource*>& lights = scene->getLights();
			for(std::list<Lightsource*>::iterator it = lights.begin(); it != lights.end(); ++it) {
				switch ((*it)->type) {
				case LT_POINT:
				{
					PointLight* light = (PointLight*)(*it);
					bool isLighted = findLightOccluder(closestIntersection, light->pos, closestObj->id);

					// if lighted, compute self-light contribution with blinn-phong
					if(isLighted)
						color += s * shadeBlinnPhong(light, ray, closestIntersection, closestObj);
					else
						color += s * closestObj->getColor(closestIntersection) * closestObj->material.ambient;
					break;
				} // case LT_POINT
				case LT_AREA:
				{
					AreaLight* light = (AreaLight*)(*it);
					std::list<glm::vec3>* samplingPositions = light->getSamplingPositions();
					std::list<glm::vec3>::iterator it;

					for(it = samplingPositions->begin(); it != samplingPositions->end(); ++it) {
						bool isLighted = findLightOccluder(closestIntersection, *(it), closestObj->id);

						if(isLighted) {
							PointLight p((*it), light->color);
							color += s * shadeBlinnPhong(&p, ray, closestIntersection, closestObj);
						} else
							color += s * closestObj->getColor(closestIntersection) * closestObj->material.ambient;
					}

					color = color / (float)(AL_SAMPLES * AL_SAMPLES);

					delete samplingPositions;
					break;
				} // case LT_AREA
				default:
					std::cerr << "ERROR: Trying to shade from unknown light source!" << std::endl; exit(EXIT_FAILURE);
					break;
				} // switch
			}
		}
		// reflected light from other objects
		float r = closestObj->material.reflLightContrib;
		if(r > 0.0) {
			// surface normal
			glm::vec3 normalDir = closestObj->computeNormal(closestIntersection);
			// compute direction of reflected ray
			Ray reflectedRay(closestIntersection, glm::reflect(ray.dir, normalDir));
			color += r * closestObj->getColor(closestIntersection) * trace(reflectedRay, NULL, remainBounces-1, closestObj->id);
		}

		// refraction light contribution, currently only works for spheres due to
		// AABB intersection computation
		if(closestObj->type == OBJ_SPHERE) {
			r = closestObj->material.refrLightContrib;
			if(r > 0.0) {
				color += r * closestObj->getColor(closestIntersection) * refraction(ray, closestIntersection, closestObj, remainBounces);
			}
		}
		return color;
	}
	// if no objects were hit, break recursion here
	return glm::vec3(0.0f);
}

glm::vec3 Raytracer::shadeBlinnPhong(PointLight* light, Ray& observer, glm::vec3 hitPoint, Object* obj) {
	glm::vec3 lightVec = light->pos-hitPoint;
	glm::vec3 lightDir = glm::fastNormalize(lightVec);
	glm::vec3 N = obj->computeNormal(hitPoint);
	glm::vec3 H = glm::fastNormalize(lightDir-observer.dir);

	glm::vec3 color = obj->getColor(hitPoint);

	// ambient color
	glm::vec3 amb = obj->material.ambient * color * light->color;

	// diffuse color
	float LdotN = MAX(glm::dot(lightDir, N), 0.0);
	glm::vec3 diff = obj->material.diffuse * color * light->color * LdotN;// / M_PI;

	// specular component, scale by 4.0 to get nice visible highlights
	float HdotN = MAX(glm::dot(H, N), 0.0);
	glm::vec3 spec = obj->material.specular * color * light->color * pow(HdotN, 4.0 * obj->material.shininess);// * ((4.0f*obj->material.shininess+8.0f)/(8.0f*(float)M_PI));

	return glm::clamp(amb+diff+spec, 0.0f, 1.0f);
//	return amb+(diff+spec) / glm::dot(lightVec, lightVec);
}

bool Raytracer::renderScene() {
	// get the perspective parameters
	Camera* cam = scene->getCamera();

	// output container
	outImage = Mat(yres, xres, CV_8UC3, Scalar(0)).clone();

	// progress variables
	float done = 0.0, rowPercentage = 1.0/(float)yres * 100.0;
	std::string status = "Rendering using " + convertInt(omp_get_num_procs()) + " processors ... ";

	// prepare cout for nice, steady progress printing
	std::cout << std::endl << "\r" << status << std::flush;
	std::cout.precision(2);
	std::cout.setf(std::ios::fixed, std::ios::floatfield);

	// for adaptive supersampling, must remember the objects as we render along
	Object* primaryObj = NULL, *lastObj = NULL;

	// core rendering loop
	#pragma omp parallel for schedule(dynamic) firstprivate(primaryObj, lastObj)
	for(unsigned y = 0; y < yres; y++) {
		for(unsigned x = 0; x < xres; x++) {
			// construct the ray for this pixel
			Ray ray = cam->constructRayForPixel(x, y, xres, yres);

			// render primary ray
			glm::vec3 pelColor(0.0, 0.0, 0.0);
			pelColor += trace(ray, &primaryObj, RSPACE_RAY_RECURSION, PRIMARY_RAY_OBJID);

			if(useAdaptiveSSAA) {
				// cast more rays to supersample if a new object was hit
				if(primaryObj != lastObj) {
					lastObj = primaryObj;
					// send more rays around the first one
					for(int tx = -2; tx < 3; tx++) {
						for(int ty = -2; ty < 3; ty++) {
							if(tx == 0 && ty == 0)
								continue;
							ray = cam->constructRayForPixel((float)x + (float)tx/5.0f, (float)y + (float)ty/5.0f, xres, yres);
							pelColor += trace(ray, &primaryObj, RSPACE_RAY_RECURSION, PRIMARY_RAY_OBJID);
						}
					}
					pelColor *= 1.0f/25.0f;
				}
			}

			if(GAMMAANDTONEMAP) {
				// go back to sRGB and tonemap with Reinhard
				pelColor = Color::delinearize(Color::tonemap(pelColor));
			} else {
				// just clamp to range
				pelColor = glm::clamp(pelColor, 0.0, 1.0);
			}
			// write to OpenCV BGR representation
			outImage.at<Vec3b>(yres-y-1,x)[2] = (unsigned char)(pelColor.r * 255.0 + 0.5);
			outImage.at<Vec3b>(yres-y-1,x)[1] = (unsigned char)(pelColor.g * 255.0 + 0.5);
			outImage.at<Vec3b>(yres-y-1,x)[0] = (unsigned char)(pelColor.b * 255.0 + 0.5);
		}
		// print progress info
		#pragma omp critical
		{
			done += rowPercentage;
			std::cout << "\r" << status << done << "%";
		}
	}

	std::cout << "\r" << status << "done     " << std::endl;
	return true;
}
