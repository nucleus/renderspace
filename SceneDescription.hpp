/*
 * SceneDescription.hpp
 *
 *  Created on: 23.12.2012
 *      Author: michael
 */

#ifndef SCENEDESCRIPTION_HPP_
#define SCENEDESCRIPTION_HPP_

#include <list>
#include <string>
#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <omp.h>

#include "glm.hpp"
#include "Material.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "Color.hpp"
#include "KdTree.hpp"

#define OFF_READER_BUF 16

/*
 * Object identifier. Can be sphere, AABB, (axis-aligned) plane or triangle.
 */
enum ObjType {
	OBJ_SPHERE = 1,
	OBJ_AABB,
	OBJ_PLANE,
	OBJ_TRIANGLE
};

class AABB; // required for obj-box intersection method
/*!
 * Generalized geometric object type.
 */
class Object {
public:
	unsigned long long id;
	ObjType type;
	Material material;

	long long* rayIDs;
	float* intersectionTestResults;

	/*
	 * Returns the materials color at a given surface point. Really, the specific point
	 * only matters for textured objects, where a texture lookup is performed to retrieve
	 * the color value; all other objs are uniformly colored.
	 */
	virtual glm::vec3 getColor(glm::vec3& surfacePoint) { return color; }

	/*
	 * Computes the objects surface normal in a given surface point. By convention, normals always
	 * point outside the object.
	 */
	virtual glm::vec3 computeNormal(glm::vec3& surfacePoint) = 0;

	/*
	 * Prints the objects parameters.
	 */
	virtual void print(std::ostream& stream) = 0;

	/*
	 * Key method for rendering. Intersects this object with a given ray and returns a boolean
	 * to specify if they intersect. If return is true, the surface point in which the ray hits
	 * the object is returned in hitPoint, otherwise, its contents are undefined.
	 */
	virtual bool intersect(Ray& ray, glm::vec3& hitPoint) = 0;
	virtual bool intersect(Ray& ray, float& tIntersect) = 0;

	/*
	 * Object-box intersection code for kd-tree construction (not sure if needed though).
	 */
	virtual bool intersect(AABB& box) = 0;

	/*
	 * Computes the maximum and minimum coordinates of the object in a given dimension. Required
	 * for kd-tree construction.
	 */
	virtual glm::vec2 getMinMaxAlongDimension(char dim) = 0;
protected:
	glm::vec3 color;
};

/*!
 * A simple sphere. Has a location and a radius.
 */
class Sphere : public Object {
public:
	Sphere(glm::vec3 pos, float rad, glm::vec3 color, unsigned long long id, int matSelector, float rInd) {
		this->pos = pos;
		this->rad = rad;
		this->color = color;
		switch(matSelector) {
			case 1:
				this->material = idealDiffMaterial;
				break;
			case 2:
				this->material = idealReflMaterial;
				break;
			case 3:
				this->material = idealRefrMaterial;
				break;
		}
		this->material.refractionIndex = rInd;
		this->id = id;
		type = OBJ_SPHERE;

		rayIDs = new long long[omp_get_num_procs()];
		for(int i = 0; i < omp_get_num_procs(); i++)
					rayIDs[i] = -1;
		intersectionTestResults = new float[omp_get_num_procs()];
	}
	~Sphere() {
		delete rayIDs;
		delete intersectionTestResults;
	}

	glm::vec3 pos;
	float rad;

	virtual glm::vec3 computeNormal(glm::vec3& surfacePoint);
	virtual void print(std::ostream& stream);
	bool intersect(Ray& ray, glm::vec3& hitPoint);
	bool intersect(Ray& ray, float& tIntersect);
	bool intersect(AABB& box);
	glm::vec2 getMinMaxAlongDimension(char dim);
};

/*!
 * Some defines for AABB computations
 */
#define NUMDIM	3
#define RIGHT	0.0
#define LEFT	1.0
#define MIDDLE	2.0
#define EPS 0.0001f
/*!
 * Axis-aligned bounding box. Defined by minimum and maximum corner coordinates.
 */
class AABB : public Object {
public:
	AABB(glm::vec3 minCorner, glm::vec3 maxCorner) {
		this->minCorner = minCorner;
		this->maxCorner = maxCorner;
		this->center = 0.5f * (minCorner + maxCorner);
		this->extents = maxCorner - minCorner;
		rayIDs = NULL;
		intersectionTestResults = NULL;
		type = OBJ_AABB;
	}

	AABB(glm::vec3 minCorner, glm::vec3 maxCorner, glm::vec3 color, unsigned long long id) {
		this->minCorner = minCorner;
		this->maxCorner = maxCorner;
		this->center = 0.5f * (minCorner + maxCorner);
		this->extents = maxCorner - minCorner;
		this->color = color;
		this->material = idealDiffMaterial;
		this->id = id;
		type = OBJ_AABB;

		rayIDs = new long long[omp_get_num_procs()];
		for(int i = 0; i < omp_get_num_procs(); i++)
			rayIDs[i] = -1;
		intersectionTestResults = new float[omp_get_num_procs()];
	}
	~AABB() {
		if(rayIDs) {
			delete rayIDs;
			rayIDs = NULL;
		}
		if(intersectionTestResults) {
			delete intersectionTestResults;
			intersectionTestResults = NULL;
		}
	}

	glm::vec3 minCorner, maxCorner, center, extents;

	virtual glm::vec3 computeNormal(glm::vec3& surfacePoint);
	virtual void print(std::ostream& stream);
	bool intersect(Ray& ray, glm::vec3& hitPoint);
	bool intersect(Ray& ray, float& tIntersect);
	bool intersect(AABB& box);
	glm::vec2 getMinMaxAlongDimension(char dim);

	float getw() { return extents.x; }
	float geth() { return extents.y; }
	float getd() { return extents.z; }

	void operator=(AABB& b);
};

/*!
 * An infinite plane. Supports regular texturing with NN-sampling (and, soon, bi/trilinear).
 */
class Plane : public Object {
public:
	Plane(glm::vec3 anchor, glm::vec3 normal, std::string texFileName, unsigned long long id);
	~Plane();

	glm::vec3 anchor, normal, texUDir, texVDir;
	std::string texFileName;
	cv::Mat* texture;

	// TODO: this does not work for both possible directions
	virtual glm::vec3 computeNormal(glm::vec3& surfacePoint) { return normal; }
	virtual void print(std::ostream& stream);
	bool intersect(Ray& ray, glm::vec3& hitPoint);
	bool intersect(Ray& ray, float& tIntersect);
	bool intersect(AABB& box);
	glm::vec3 getColor(glm::vec3& surfacePoint);
	glm::vec2 getMinMaxAlongDimension(char dim);
	float distance(glm::vec3 p) { return glm::dot(normal, p-anchor); }
};

/*!
 * A triangle. Uses barycentric coordinates to compute intersections. Has pointers to
 * its three vertices and the corresponding vertex normals.
 */
class Triangle : public Object {
public:
	Triangle(glm::vec3* v1, glm::vec3* v2, glm::vec3* v3, glm::vec3* n1, glm::vec3* n2, glm::vec3* n3, glm::vec3 color, unsigned long long id);
	~Triangle();

	glm::vec3* vertices[3];
	glm::vec3* normals[3];
	glm::vec3 faceNormal;
	float nu, nv, nd, bnu, bnv, cnu, cnv;
	int k;
	float *U, *V;

	glm::vec3& getFaceNormal() { return faceNormal; }
	virtual glm::vec3 computeNormal(glm::vec3& surfacePoint);

	virtual void print(std::ostream& stream);
	bool intersect(Ray& ray, glm::vec3& hitPoint);
	bool intersect(Ray& ray, float& tIntersect);
	bool intersect(AABB& box);
	bool planeBoxOverlap( glm::vec3& norm, glm::vec3& vert, glm::vec3& maxBox );
	bool intersectTriBox( glm::vec3& a_BoxCentre, glm::vec3& a_BoxHalfsize, glm::vec3& a_V0, glm::vec3& a_V1, glm::vec3& a_V2 );
	glm::vec2 getMinMaxAlongDimension(char dim);
};

class KdTreeNode;
class KdTree;
/*!
 * A container for all the geometric objects and shapes that make up the 3D world.
 */
class SceneDescription {
public:
	SceneDescription();
	~SceneDescription();

	bool readSceneFile(std::string fname);
	void dumpScene();

	void buildKdTree();
	KdTree* getKdTree();

	bool loadMeshOFF(std::string fname, std::list<Object*>& objs, glm::vec3 meshCol, int material);

	void insertObj(Object* obj) { objects.push_back(obj); }

	Camera* getCamera() { return cam; }
	glm::vec3 getMinPoint() { return minPoint; }
	glm::vec3 getMaxPoint() { return maxPoint; }
	std::list<Object*>& getObjects() { return objects; }
	std::list<Lightsource*>& getLights() { return lights; }
private:
	KdTree* kdtree;

	std::list<Object*> objects;
	std::list<Lightsource*> lights;
	Camera* cam;

	glm::vec3 camPos, camUp, camAt;

	glm::vec3 minPoint, maxPoint;

	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> vertexNormals;

	unsigned long long globalObjID;
};

#endif /* SCENEDESCRIPTION_HPP_ */
