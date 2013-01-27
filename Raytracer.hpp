/*
 * Raytracer.hpp
 *
 *  Created on: 23.12.2012
 *      Author: michael
 */

#include "Renderer.hpp"
#include "glm.hpp"
#include "Ray.hpp"

#ifndef RAYTRACER_HPP_
#define RAYTRACER_HPP_

/* some global defines for raytracing, such as recursion depth */
#define RSPACE_RAY_RECURSION 	((int)3)
#define RSPACE_RAY_REFL_CONTRIB	((float)0.5)
#define PRIMARY_RAY_OBJID 		((long long)-1)

/*
 *	The raytracer core engine.
 */
class Raytracer : public Renderer {
public:
	Raytracer() : Renderer() {
		std::cout << "Loading raytracer module ... done" << std::endl;
	}
	virtual bool renderScene();
private:
	/*
	 * Camera setup
	 */
	glm::vec3 camPos, camAt, camUp;

	/*
	 * Function: trace
	 *
	 * This is the core function to trace rays recursively. RemainBounces is decremented on each ray bounce and
	 * will eventually stop the recursion when it becomes zero. The ID of the source object of the ray is required
	 * to prevent reflection acne.
	 */
	glm::vec3 trace(Ray& ray, Object** primaryObj, int remainBounces, unsigned long long sourceObjID);

	/*
	 * Function: findLightOccluder
	 *
	 * Tests if the light ray between the points start (on object with ID excludeID) and target is blocked by a non-dielecric object.
	 */
	bool findLightOccluder(glm::vec3& start, glm::vec3& target, unsigned long long excludeID);

	/*
	 * Function: findNearest
	 *
	 * Finds the nearest geometry intersection point between a ray and the scene.
	 */
	bool findNearest(Ray& ray, glm::vec3& hit, Object** closestObj, unsigned long long excludeID, float maxDist);

	/*
	 * Function: refraction
	 *
	 * Helper function to compute color contribution from refracted rays.
	 */
	glm::vec3 refraction(Ray& ray, glm::vec3& hitPoint, Object* obj, int remainBounces);

	/*
	 * Function: computeBlinnPhonLightColor
	 *
	 * Computes the color of a surface point according to the lambertian and blinn-phong lighting models.
	 * I.e., total light is ambient + diffuse (lambert) + specular (blinnphong) using half vectors.
	 */
	glm::vec3 shadeBlinnPhong(PointLight* light, Ray& observer, glm::vec3 hitPoint, Object* obj);
};

#endif /* RAYTRACER_HPP_ */
