/*
 * SceneDescription.cpp
 *
 *  Created on: 23.12.2012
 *      Author: michael
 */

#include <fstream>
#include <cmath>

#include "SceneDescription.hpp"
#include "gtx/fast_square_root.hpp"

int totalIntersectionTests = 0;
char mod[5] = {0, 1, 2, 0, 1};

/*/////////////////////////////////////////////////////////////////
 *//////////////////////// SPHERE /////////////////////////////////
 ////////////////////////////////////////////////////////////////*/

glm::vec3 Sphere::computeNormal(glm::vec3& surfacePoint) {
	return glm::normalize(surfacePoint-pos);
}

void Sphere::print(std::ostream& stream) {
	stream << "Sphere - ObjID(" << id << "): At (";
	stream << pos.x << "," << pos.y << "," << pos.z << "), radius (";
	stream << rad << ")" << std::endl;
}

bool Sphere::intersect(Ray& ray, glm::vec3& hitPoint) {
	//Compute A, B and C coefficients
	float a = glm::dot(ray.dir, ray.dir);
	float b = 2.0 * glm::dot(ray.dir, ray.org-pos);
	float c = glm::dot(ray.org-pos, ray.org-pos) - (rad * rad);

	// Find discriminant
	float disc = b * b - 4 * a * c;

	// if discriminant is negative there are no real roots, so return
	// false as ray misses sphere
	if (disc < 0)
		return false;

	// compute q
	float distSqrt = sqrt(disc);
	float q;
	if (b < 0)
		q = (-b - distSqrt)/2.0;
	else
		q = (-b + distSqrt)/2.0;

	// compute t0 and t1
	float t0 = q / a;
	float t1 = c / q;

	// make sure t0 is smaller than t1
	if (t0 > t1) {
		// if t0 is bigger than t1 swap them around
		float temp = t0;
		t0 = t1;
		t1 = temp;
	}

	// if t1 is less than zero, the object is in the ray's negative direction
	// and consequently the ray misses the sphere
	if (t1 < 0)
		return false;

	// if t0 is less than zero, the intersection point is at t1
	if (t0 < 0) {
		hitPoint = ray.org + t1 * ray.dir;
		return true;
	} else { // else the intersection point is at t0
		hitPoint = ray.org + t0 * ray.dir;
		return true;
	}
}

bool Sphere::intersect(Ray& ray, float& tIntersect) {
	totalIntersectionTests++;
	//Compute A, B and C coefficients
	float a = glm::dot(ray.dir, ray.dir);
	float b = 2.0 * glm::dot(ray.dir, ray.org-pos);
	float c = glm::dot(ray.org-pos, ray.org-pos) - (rad * rad);

	// Find discriminant
	float disc = b * b - 4 * a * c;

	// if discriminant is negative there are no real roots, so return
	// false as ray misses sphere
	if (disc < 0)
		return false;

	// compute q
	float distSqrt = sqrt(disc);
	float q;
	if (b < 0)
		q = (-b - distSqrt)/2.0;
	else
		q = (-b + distSqrt)/2.0;

	// compute t0 and t1
	float t0 = q / a;
	float t1 = c / q;

	// make sure t0 is smaller than t1
	if (t0 > t1) {
		// if t0 is bigger than t1 swap them around
		float temp = t0;
		t0 = t1;
		t1 = temp;
	}

	// if t1 is less than zero, the object is in the ray's negative direction
	// and consequently the ray misses the sphere
	if (t1 < 0)
		return false;

	// if t0 is less than zero, the intersection point is at t1
	if (t0 < 0) {
		tIntersect = t1;
		return true;
	} else { // else the intersection point is at t0
		tIntersect = t0;
		return true;
	}
}


bool Sphere::intersect(AABB& box) {
	float dmin = 0;
	glm::vec3 spos = this->pos;
	glm::vec3 bpos = box.minCorner;
	glm::vec3 bsize = box.extents;
	for(int i = 0; i < 3; i++) {
		if (spos[i] < bpos[i]) {
			dmin = dmin + (spos[i] - bpos[i]) * (spos[i] - bpos[i]);
		} else if (spos[i] > (bpos[i] + bsize[i])) {
			dmin = dmin + (spos[i] - (bpos[i] + bsize[i])) * (spos[i] - (bpos[i] + bsize[i]));
		}
	}
	return (dmin <= rad*rad);
}

glm::vec2 Sphere::getMinMaxAlongDimension(char dim) {
	return glm::vec2(pos[dim]-rad, pos[dim]+rad);
}


/*/////////////////////////////////////////////////////////////////
 *////////////////////////// AABB /////////////////////////////////
 ////////////////////////////////////////////////////////////////*/

glm::vec3 AABB::computeNormal(glm::vec3& surfacePoint) {
	float curDist, minDist = (float)1e10;
	glm::vec3 normal(0.0f);
	// minimum corner tests
	if((curDist = ABS(surfacePoint.x-minCorner.x)) < minDist) {
		minDist = curDist;
		normal = glm::vec3(-1.0f, 0.0, 0.0);
	}
	if((curDist = ABS(surfacePoint.y-minCorner.y)) < minDist) {
		minDist = curDist;
		normal =  glm::vec3(0.0, -1.0f, 0.0);
	}
	if((curDist = ABS(surfacePoint.z-minCorner.z)) < minDist) {
		minDist = curDist;
		normal = glm::vec3(0.0, 0.0, -1.0f);
	}
	// maximum corner tests
	if((curDist = ABS(surfacePoint.x-maxCorner.x)) < minDist) {
		minDist = curDist;
		normal = glm::vec3(1.0f, 0.0, 0.0);
	}
	if((curDist = ABS(surfacePoint.y-maxCorner.y)) < minDist) {
		minDist = curDist;
		normal = glm::vec3(0.0, 1.0f, 0.0);
	}
	if((curDist = ABS(surfacePoint.z-maxCorner.z)) < minDist) {
		minDist = curDist;
		normal = glm::vec3(0.0, 0.0, 1.0f);
	}
	return normal;
}

void AABB::print(std::ostream& stream) {
	stream << "AABB - ObjID(" << id << "): minCorner (";
	stream << minCorner.x << "," << minCorner.y << "," << minCorner.z << "), maxCorner (";
	stream << maxCorner.x << "," << maxCorner.y << "," << maxCorner.z << ")" << std::endl;
}

bool AABB::intersect(Ray& ray, glm::vec3& hitPoint) {
	bool inside = true;
	glm::vec3 quadrant, maxT, candidatePlane;
	int whichPlane;

	/* Find candidate planes; this loop can be avoided if
   	rays cast all from the eye(assume perspective view) */
	for (int i = 0; i < NUMDIM; i++) {
		if(ray.org[i] < minCorner[i]) {
			quadrant[i] = LEFT;
			candidatePlane[i] = minCorner[i];
			inside = false;
		} else if (ray.org[i] > maxCorner[i]) {
			quadrant[i] = RIGHT;
			candidatePlane[i] = maxCorner[i];
			inside = false;
		} else{
			quadrant[i] = MIDDLE;
		}
	}

	/* Ray origin inside bounding box */
	if(inside)	{
		hitPoint = ray.org;
		return true;
	}

	/* Calculate T distances to candidate planes */
	for (int i = 0; i < NUMDIM; i++)
		if (quadrant[i] != MIDDLE && ray.dir[i] != 0.0)
			maxT[i] = (candidatePlane[i]-ray.org[i]) / ray.dir[i];
		else
			maxT[i] = -1.0;

	/* Get largest of the maxT's for final choice of intersection */
	whichPlane = 0;
	for (int i = 1; i < NUMDIM; i++)
		if (maxT[whichPlane] < maxT[i])
			whichPlane = i;

	/* Check final candidate actually inside box */
	if (maxT[whichPlane] < 0.0)
		return false;
	for (int i = 0; i < NUMDIM; i++)
		if (whichPlane != i) {
			hitPoint[i] = ray.org[i] + maxT[whichPlane] * ray.dir[i];
			if (hitPoint[i] < minCorner[i] || hitPoint[i] > maxCorner[i])
				return false;
		} else {
			hitPoint[i] = candidatePlane[i];
		}
	return true;				/* ray hits box */
}

bool AABB::intersect(Ray& ray, float& tIntersect) {
	totalIntersectionTests++;
	float dist[6];
	glm::vec3 ip[6], d = ray.dir, o = ray.org;
	bool retval = false;
	for ( int i = 0; i < 6; i++ ) dist[i] = -1;
	glm::vec3 v1 = minCorner, v2 = maxCorner;
	if (d.x) {
		float rc = 1.0f / d.x;
		dist[0] = (v1.x - o.x) * rc;
		dist[3] = (v2.x - o.x) * rc;
	}
	if (d.y) {
		float rc = 1.0f / d.y;
		dist[1] = (v1.y - o.y) * rc;
		dist[4] = (v2.y - o.y) * rc;
	}
	if (d.z) {
		float rc = 1.0f / d.z;
		dist[2] = (v1.z - o.z) * rc;
		dist[5] = (v2.z - o.z) * rc;
	}
	float maxDist = 1e10;
	for (int i = 0; i < 6; i++ ) if (dist[i] > 0) {
		ip[i] = o + dist[i] * d;
		if ((ip[i].x > (v1.x - EPS)) && (ip[i].x < (v2.x + EPS)) &&
			(ip[i].y > (v1.y - EPS)) && (ip[i].y < (v2.y + EPS)) &&
			(ip[i].z > (v1.z - EPS)) && (ip[i].z < (v2.z + EPS))) {
			if (dist[i] < maxDist) {
				maxDist = dist[i];
				retval = true;
			}
		}
	}
	tIntersect = maxDist;
	return retval;
}

bool AABB::intersect(AABB & box) {
    glm::vec3 T = box.minCorner - minCorner;//vector from A to B
    return 	fabs(T.x) <= (extents.x + box.extents.x)
    	 && fabs(T.y) <= (extents.y + box.extents.y)
    	 && fabs(T.z) <= (extents.z + box.extents.z);
}

glm::vec2 AABB::getMinMaxAlongDimension(char dim) {
	return glm::vec2(minCorner[dim], maxCorner[dim]);
}

void AABB::operator=(AABB& b) {
	minCorner = b.minCorner;
	maxCorner = b.maxCorner;
	extents = b.extents;
	center = b.center;
	type = OBJ_AABB;
}

/*/////////////////////////////////////////////////////////////////
 *///////////////////////////// Plane /////////////////////////////
 ////////////////////////////////////////////////////////////////*/
#if 0
static double myRound(double num, unsigned digits) {
    double v[] = {1, 10, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8};
	if(digits > (sizeof(v)/sizeof(double))) return num;
    return floor(num * v[digits] + 0.5) / v[digits];
}

static cv::Vec3b interpolateLinear(cv::Vec3b& a, cv::Vec3b& b, float weight) {
	cv::Vec3b final;
	final[0] = a[0] * (1.0-weight) + b[0] * weight;
	final[1] = a[1] * (1.0-weight) + b[1] * weight;
	final[2] = a[2] * (1.0-weight) + b[2] * weight;
	return final;
}

static cv::Vec3b filterBilinear(float u, float v, cv::Vec3b& ul, cv::Vec3b& ur, cv::Vec3b& ll, cv::Vec3b& lr) {
	float x_weight = myRound(v - floor(v), 3);
	float y_weight = myRound(u - floor(u), 3);

	cv::Vec3b sample_v_upper = interpolateLinear(ul, ur, x_weight);
	cv::Vec3b sample_v_lower = interpolateLinear(ll, lr, x_weight);
	cv::Vec3b sample_h = interpolateLinear(sample_v_upper, sample_v_lower, y_weight);

	return sample_h;
}
#endif

static cv::Mat* bindTexture(std::string texFileName) {
	cv::Mat temp = cv::imread(texFileName.c_str(), CV_LOAD_IMAGE_COLOR);
	cv::Mat* tex = new cv::Mat(temp.rows, temp.cols, temp.type());
	temp.copyTo(*tex);
	return tex;
}

Plane::Plane(glm::vec3 anchor, glm::vec3 normal, std::string texFileName, unsigned long long id) {
	// load texture image into the plane object
	this->texFileName = texFileName;
	this->texture = NULL;
	this->texture = bindTexture(texFileName);
	if(texture == NULL) {
		std::cerr << "ERROR: Cannot bind texture '" << texFileName << "'" << std::endl;
	}

	this->anchor = anchor;
	this->normal = glm::normalize(normal);
	this->texUDir = glm::vec3(normal.y, normal.z, -normal.x);
	this->texVDir = glm::cross(this->texUDir, this->normal);
	this->material = idealDiffMaterial;
	this->material.specular = glm::vec3(0.9f);
	this->material.reflLightContrib = 0.6;
	this->id = id;
	type = OBJ_PLANE;

	rayIDs = new long long[omp_get_num_procs()];
	for(int i = 0; i < omp_get_num_procs(); i++)
				rayIDs[i] = -1;
	intersectionTestResults = new float[omp_get_num_procs()];
}

Plane::~Plane() {
	if(texture)
		delete texture;
	delete rayIDs;
	delete intersectionTestResults;
}

void Plane::print(std::ostream& stream) {
	stream << "Plane - ObjID(" << id << "): At (";
	stream << anchor.x << "," << anchor.y << "," << anchor.z << ") /w normal (";
	stream << normal.x << "," << normal.y << "," << normal.z << "), texfile: " << texFileName << std::endl;
}

bool Plane::intersect(Ray& ray, glm::vec3& hitPoint) {
	float t = glm::dot(normal, anchor-ray.org) / glm::dot(normal, ray.dir);
	if(t > 0.0) {
		hitPoint = ray.org + t * ray.dir;
		return true;
	}
	return false;
}

bool Plane::intersect(Ray& ray, float& tIntersect) {
	totalIntersectionTests++;
	float t = glm::dot(normal, anchor-ray.org) / glm::dot(normal, ray.dir);
	if(t > 0.0) {
		tIntersect = t;
		return true;
	}
	return false;
}

bool Plane::intersect(AABB & box) {
	glm::vec3 p = box.minCorner;
	if (normal.x >= 0)
		p.x = box.maxCorner.x;
	if (normal.y >= 0)
		p.y = box.maxCorner.y;
	if (normal.z >= 0)
		p.z = box.maxCorner.z;
	glm::vec3 n = box.maxCorner;
	if (normal.x >= 0)
		n.x = box.minCorner.x;
	if (normal.y >= 0 )
		n.y = box.minCorner.y;
	if (normal.z >= 0)
		n.z = box.minCorner.z;
	return ( (distance(p) > 0) && (distance(n) < 0));
}

#define TEX_U_SCALE 40
#define TEX_V_SCALE 40

glm::vec3 Plane::getColor(glm::vec3& surfacePoint) {
	cv::Vec3b texel;

	float uscale = glm::dot(surfacePoint, texUDir) * TEX_U_SCALE;
	float vscale = glm::dot(surfacePoint, texVDir) * TEX_V_SCALE;
	while(uscale > texture->rows) uscale -= (float)texture->rows;
	while(uscale < 0) uscale += (float)texture->rows;
	while(vscale > texture->cols) vscale -= (float)texture->cols;
	while(vscale < 0) vscale += (float)texture->cols;

	unsigned u = (unsigned)uscale;
	unsigned v = (unsigned)vscale;

//	float uBi = uscale;
//	float vBi = vscale;

//#pragma omp critical
//	{
//	std::cout << u << "," << v << " : ";
//	std::cout << uBi << "," << vBi << std::endl << std::endl;
//	}

//	bool down =  (int)uBi   < texture->rows;
//	bool up =    (int)uBi+1 < texture->rows;
//	bool left =  (int)vBi   < texture->cols;
//	bool right = (int)vBi+1 < texture->cols;
//
//	if(up && down && left && right) {
//		texel = filterBilinear(	uBi, vBi,
//								texture->at<cv::Vec3b>(texture->rows-1-((int)uBi+1), (int)vBi),
//								texture->at<cv::Vec3b>(texture->rows-1-((int)uBi+1), (int)vBi+1),
//								texture->at<cv::Vec3b>(texture->rows-1-(int)uBi, (int)vBi),
//								texture->at<cv::Vec3b>(texture->rows-1-(int)uBi, (int)vBi+1));
//	} else {
//		texel = texture->at<cv::Vec3b>(texture->rows - u - 1, v);
//	}

	texel = texture->at<cv::Vec3b>(texture->rows - u - 1, v);

	glm::vec3 color;
	color.r = (float)texel[2]/255.0f;
	color.g = (float)texel[1]/255.0f;
	color.b = (float)texel[0]/255.0f;
	return color;
}

// FIXME FIXME FIXME this only works on axis-aligned planes
glm::vec2 Plane::getMinMaxAlongDimension(char dim) {
	if(normal[dim] > (0.001f) || normal[dim] < (-0.001f))
		return glm::vec2(anchor[dim]-0.001f, anchor[dim]+0.001f);
	return glm::vec2((float)-1e10, (float)1e10);
}

/*/////////////////////////////////////////////////////////////////
 *////////////////////////// Triangle /////////////////////////////
 ////////////////////////////////////////////////////////////////*/

Triangle::Triangle(glm::vec3* v1, glm::vec3* v2, glm::vec3* v3, glm::vec3* n1, glm::vec3* n2, glm::vec3* n3, glm::vec3 color, unsigned long long id) {
	vertices[0] = v1;
	vertices[1] = v2;
	vertices[2] = v3;
	normals[0] = n1;
	normals[1] = n2;
	normals[2] = n3;
	type = OBJ_TRIANGLE;
	this->color = color;
	this->id = id;
	this->material = idealDiffMaterial;

	// precomputations: init
	glm::vec3 A = *v1;
	glm::vec3 B = *v2;
	glm::vec3 C = *v3;
	glm::vec3 c = B - A;
	glm::vec3 b = C - A;
	faceNormal = -glm::cross(b,c);
	int u,v;
	// get the dominant direction
	if(ABS(faceNormal.x) > ABS(faceNormal.y)) {
		if(ABS(faceNormal.x) > ABS(faceNormal.z)) k = 0; else k = 2;
	} else {
		if(ABS(faceNormal.y) > ABS(faceNormal.z)) k = 1; else k = 2;
	}

	u = (k+1)%3;
	v = (k+2)%3;

	// precomputations: do it
	float krec = 1.0f / faceNormal[k];
	nu = faceNormal[u] * krec;
	nv = faceNormal[v] * krec;
	nd = glm::dot(faceNormal, A) * krec;

	float reci = 1.0f / (b[u] * c[v] - b[v] * c[u]);
	bnu = b[u] * reci;
	bnv = -b[v] * reci;

	cnu = c[v] * reci;
	cnv = -c[u] * reci;

	// normalize this triangles face normal and add it to the three vertex normals
	faceNormal = glm::normalize(faceNormal);
	*(normals[0]) += faceNormal;
	*(normals[1]) += faceNormal;
	*(normals[2]) += faceNormal;

	rayIDs = new long long[omp_get_num_procs()];
	for(int i = 0; i < omp_get_num_procs(); i++)
				rayIDs[i] = -1;
	intersectionTestResults = new float[omp_get_num_procs()];
	U = new float[omp_get_num_procs()];
	V = new float[omp_get_num_procs()];
}

Triangle::~Triangle() {
	delete rayIDs;
	delete intersectionTestResults;
	delete U;
	delete V;
}

bool Triangle::intersect(Ray& ray, float& tIntersect) {
	totalIntersectionTests++;
	#define ku mod[k+1]
	#define kv mod[k+2]
	glm::vec3 O = ray.org, D = ray.dir, A = *(vertices[0]);
	float dnr = (D[k] + nu * D[ku] + nv * D[kv]);
//	if(ABS(dnr) < 0.0001f) return false;
	float lnd = 1.0f / dnr;
	float t = (nd - O[k] - nu * O[ku] - nv * O[kv]) * lnd;
	if(t <= 0.0f) return false;
	float hu = O[ku] + t * D[ku] - A[ku];
	float hv = O[kv] + t * D[kv] - A[kv];
	float beta = hv * bnu + hu * bnv;
	if(beta < 0.0f) return false;
	U[omp_get_thread_num()] = beta;
	float gamma = hu * cnu + hv * cnv;
	if(gamma < 0.0f) return false;
	V[omp_get_thread_num()] = gamma;
	if((beta + gamma) > 1.0f) return false;
	tIntersect = t;
	return true;
}

bool Triangle::intersect(Ray& ray, glm::vec3& hitPoint) {
	float t;
	bool intersected = intersect(ray, t);
	if(!intersected) return false;
	hitPoint = ray.org + t * ray.dir;
	return true;
}

// TODO: integrate a U-V-cache private to each worker thread into the triangle class
glm::vec3 Triangle::computeNormal(glm::vec3& surfacePoint) {
	return faceNormal;
//	glm::vec3 N1 = *(normals[0]);
//	glm::vec3 N2 = *(normals[1]);
//	glm::vec3 N3 = *(normals[2]);
//	glm::vec3 ret = N1 + U[omp_get_thread_num()] * (N2 - N1) + V[omp_get_thread_num()] * (N3 - N1);
//	return glm::fastNormalize(ret);
}

glm::vec2 Triangle::getMinMaxAlongDimension(char dim) {
	glm::vec2 ret;
	glm::vec3 pos1 = *(vertices[0]);
	ret.x = pos1[dim], ret.y = pos1[dim];
	for(int i = 1; i < 3; i++) {
		glm::vec3 pos = *(vertices[i]);
		if(pos[dim] < ret.x) ret.x = pos[dim];
		if(pos[dim] > ret.y) ret.y = pos[dim];
	}
	return ret;
}

#define FINDMINMAX( x0, x1, x2, min, max ) \
  min = max = x0; if(x1<min) min=x1; if(x1>max) max=x1; if(x2<min) min=x2; if(x2>max) max=x2;
// X-tests
#define AXISTEST_X01( a, b, fa, fb )											\
	p0 = a * v0[1] - b * v0[2], p2 = a * v2[1] - b * v2[2]; \
    if (p0 < p2) { min = p0; max = p2;} else { min = p2; max = p0; }			\
	rad = fa * a_BoxHalfsize[1] + fb * a_BoxHalfsize[2];				\
	if (min > rad || max < -rad) return 0;
#define AXISTEST_X2( a, b, fa, fb )												\
	p0 = a * v0[1] - b * v0[2], p1 = a * v1[1] - b * v1[2];	\
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0;}			\
	rad = fa * a_BoxHalfsize[1] + fb * a_BoxHalfsize[2];				\
	if(min>rad || max<-rad) return 0;
// Y-tests
#define AXISTEST_Y02( a, b, fa, fb )											\
	p0 = -a * v0[0] + b * v0[2], p2 = -a * v2[0] + b * v2[2]; \
    if(p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; }			\
	rad = fa * a_BoxHalfsize[0] + fb * a_BoxHalfsize[2];				\
	if (min > rad || max < -rad) return 0;
#define AXISTEST_Y1( a, b, fa, fb )												\
	p0 = -a * v0[0] + b * v0[2], p1 = -a * v1[0] + b * v1[2]; \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }			\
	rad = fa * a_BoxHalfsize[0] + fb * a_BoxHalfsize[2];				\
	if (min > rad || max < -rad) return 0;
// Z-tests
#define AXISTEST_Z12( a, b, fa, fb )											\
	p1 = a * v1[0] - b * v1[1], p2 = a * v2[0] - b * v2[1]; \
    if(p2 < p1) { min = p2; max = p1; } else { min = p1; max = p2; }			\
	rad = fa * a_BoxHalfsize[0] + fb * a_BoxHalfsize[1];				\
	if (min > rad || max < -rad) return 0;
#define AXISTEST_Z0( a, b, fa, fb )												\
	p0 = a * v0[0] - b * v0[1], p1 = a * v1[0] - b * v1[1];	\
    if(p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }			\
	rad = fa * a_BoxHalfsize[0] + fb * a_BoxHalfsize[1];				\
	if (min > rad || max < -rad) return 0;

bool Triangle::planeBoxOverlap( glm::vec3& norm, glm::vec3& vert, glm::vec3& maxBox ) {
	glm::vec3 vmin, vmax;
	for( int q = 0; q < 3; q++ ) {
		float v = vert[q];
		if (norm[q] > 0.0f) {
			vmin[q] = -maxBox[q] - v;
			vmax[q] =  maxBox[q] - v;
		} else {
			vmin[q] =  maxBox[q] - v;
			vmax[q] = -maxBox[q] - v;
		}
	}
	if (glm::dot(norm, vmin) > 0.0f) return false;
	if (glm::dot(norm, vmax) >= 0.0f) return true;
	return false;
}

bool Triangle::intersectTriBox(glm::vec3& a_BoxCentre, glm::vec3& a_BoxHalfsize, glm::vec3& a_V0, glm::vec3& a_V1, glm::vec3& a_V2) {
	glm::vec3 v0, v1, v2, normal, e0, e1, e2;
	float min, max, p0, p1, p2, rad, fex, fey, fez;
	v0 = a_V0 - a_BoxCentre;
	v1 = a_V1 - a_BoxCentre;
	v2 = a_V2 - a_BoxCentre;
	e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;
	fex = fabsf( e0[0] );
	fey = fabsf( e0[1] );
	fez = fabsf( e0[2] );
	AXISTEST_X01( e0[2], e0[1], fez, fey );
	AXISTEST_Y02( e0[2], e0[0], fez, fex );
	AXISTEST_Z12( e0[1], e0[0], fey, fex );
	fex = fabsf( e1[0] );
	fey = fabsf( e1[1] );
	fez = fabsf( e1[2] );
	AXISTEST_X01( e1[2], e1[1], fez, fey );
	AXISTEST_Y02( e1[2], e1[0], fez, fex );
	AXISTEST_Z0 ( e1[1], e1[0], fey, fex );
	fex = fabsf( e2[0] );
	fey = fabsf( e2[1] );
	fez = fabsf( e2[2] );
	AXISTEST_X2 ( e2[2], e2[1], fez, fey );
	AXISTEST_Y1 ( e2[2], e2[0], fez, fex );
	AXISTEST_Z12( e2[1], e2[0], fey, fex );
	FINDMINMAX( v0[0], v1[0], v2[0], min, max );
	if (min > a_BoxHalfsize[0] || max < -a_BoxHalfsize[0]) return false;
	FINDMINMAX( v0[1], v1[1], v2[1], min, max );
	if (min > a_BoxHalfsize[1] || max < -a_BoxHalfsize[1]) return false;
	FINDMINMAX( v0[2], v1[2], v2[2], min, max );
	if (min > a_BoxHalfsize[2] || max < -a_BoxHalfsize[2]) return false;
	normal = glm::cross(e0,e1);//e0.Cross( e1 );
	if (!planeBoxOverlap( normal, v0, a_BoxHalfsize )) return false;
	return true;
}

bool Triangle::intersect(AABB& box) {
	glm::vec3 v1 = vertices[0][0], v2 = vertices[1][0], v3 = vertices[2][0];
	glm::vec3 halfBoxSize = 0.5f * box.extents;
	glm::vec3 center = box.center;
	return intersectTriBox(center, halfBoxSize, v1, v2, v3);
}

void Triangle::print(std::ostream& stream) {
	glm::vec3 v1 = *(vertices[0]), v2 = *(vertices[1]), v3 = *(vertices[2]);
	stream << "Triangle - ObjID(" << id << "): Vertices (";
	stream << v1.x << "," << v1.y << "," << v1.z << "), (";
	stream << v2.x << "," << v2.y << "," << v2.z << "), (";
	stream << v3.x << "," << v3.y << "," << v3.z << ")" << std::endl;
}

/*/////////////////////////////////////////////////////////////////
 *////////////////////// SceneDescription /////////////////////////
 ////////////////////////////////////////////////////////////////*/

SceneDescription::SceneDescription() {
	globalObjID = 0;
	cam = NULL;
}

SceneDescription::~SceneDescription() {
	for(std::list<Object*>::iterator it = objects.begin(); it != objects.end(); ++it)
		delete (*it);
	for(std::list<Lightsource*>::iterator it = lights.begin(); it != lights.end(); ++it)
		delete (*it);
	if(cam)
		delete cam;
	if(kdtree)
		delete kdtree;
}

bool SceneDescription::readSceneFile(std::string fname) {
	std::fstream in(fname.c_str());

	unsigned numObjects, numLights;
	unsigned long long objtype;
	float x,x1,x2,y,y1,y2,z,z1,z2,rad,r,g,b;
	glm::vec3 col;
	int materialSel, fov;
	std::string texfile;

	std::cout << "Reading scene file " << fname << " ...";

	if(in) {
		// get the number of objects and read them
		in >> numObjects >> numLights;
		for(unsigned i = 0; i < numObjects; i++) {
			in >> objtype;

			// depending on the object type, the parameters have different meanings
			switch(objtype) {
				case OBJ_SPHERE:
					in >> x >> y >> z >> rad >> r >> g >> b >> materialSel;
					if(materialSel == 3) // if refractive material, get refraction index
						in >> x1;
					col = GAMMAANDTONEMAP ? Color::linearize(glm::vec3(r,g,b)) : glm::vec3(r,g,b);
					objects.push_back(new Sphere(glm::vec3(x,y,z), rad, col, globalObjID++, materialSel, x1));
					minPoint.x = std::min(minPoint.x, x-rad); maxPoint.x = std::max(maxPoint.x, x+rad);
					minPoint.y = std::min(minPoint.y, y-rad); maxPoint.y = std::max(maxPoint.y, y+rad);
					minPoint.z = std::min(minPoint.z, z-rad); maxPoint.z = std::max(maxPoint.z, z+rad);
					break;
				case OBJ_AABB:
					in >> x >> y >> z >> x1 >> y1 >> z1 >> r >> g >> b;
					col = GAMMAANDTONEMAP ? Color::linearize(glm::vec3(r,g,b)) : glm::vec3(r,g,b);
					objects.push_back(new AABB(glm::vec3(x,y,z), glm::vec3(x1,y1,z1), col, globalObjID++));
					minPoint.x = std::min(minPoint.x, x); maxPoint.x = std::max(maxPoint.x, x1);
					minPoint.y = std::min(minPoint.y, y); maxPoint.y = std::max(maxPoint.y, y1);
					minPoint.z = std::min(minPoint.z, z); maxPoint.z = std::max(maxPoint.z, z1);
					break;
				case OBJ_PLANE:
					in >> x >> y >> z >> x1 >> y1 >> z1 >> texfile;
					if(!((x1 == 0.0 && y1 == 0.0) || (x1 == 0.0 && z1 == 0.0) || (y1 == 0.0 && z1 == 0.0))) {
						std::cerr << "ERROR: kd-tree only works with axis-aligned planes!" << std::endl; exit(EXIT_FAILURE);
					}
					objects.push_back(new Plane(glm::vec3(x,y,z), glm::vec3(x1,y1,z1), texfile, globalObjID++));
					break;
//				case OBJ_TRIANGLE:
//					in >> x >> y >> z >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> r >> g >> b;
//					col = GAMMAANDTONEMAP ? Color::linearize(glm::vec3(r,g,b)) : glm::vec3(r,g,b);
//					objects.push_back(new Triangle(new glm::vec3(x,y,z), new glm::vec3(x1,y1,z1), new glm::vec3(x2,y2,z2), col, globalObjID++));
//					minPoint.x = std::min(minPoint.x, TRIMIN(x,x1,x2)); maxPoint.x = std::max(maxPoint.x, TRIMAX(x,x1,x2));
//					minPoint.y = std::min(minPoint.y, TRIMIN(y,y1,y2)); maxPoint.y = std::max(maxPoint.y, TRIMAX(y,y1,y2));
//					minPoint.z = std::min(minPoint.z, TRIMIN(z,z1,z2)); maxPoint.z = std::max(maxPoint.z, TRIMAX(z,z1,z2));
//					break;
				case OBJ_TRIANGLE:
					in >> texfile >> r >> g >> b >> materialSel;
					if(!loadMeshOFF(texfile, objects, glm::vec3(r,g,b), materialSel)) {
						std::cerr << "ERROR: Cannot read mesh file " << texfile << std::endl; exit(EXIT_FAILURE);
					}
					break;
				default:
					std::cerr << "ERROR: Parsed unknown object type!" << std::endl; exit(EXIT_FAILURE);
					break;
			}
		}

		// read the lights
		for(unsigned i = 0; i < numLights; i++) {
			in >> objtype;

			switch(objtype) {
			case LT_POINT:
				in >> x >> y >> z >> r >> g >> b;
				col = GAMMAANDTONEMAP ? Color::linearize(glm::vec3(r,g,b)) : glm::vec3(r,g,b);
				lights.push_back(new PointLight(glm::vec3(x,y,z), col));
				break;
			case LT_AREA:
				in >> x >> y >> z >> x1 >> y1 >> z1 >> r >> g >> b;
				col = GAMMAANDTONEMAP ? Color::linearize(glm::vec3(r,g,b)) : glm::vec3(r,g,b);
				lights.push_back(new AreaLight(glm::vec3(x,y,z), glm::vec3(x1,y1,z1), col, AL_SAMPLES));
				break;
			default:
				std::cerr << "ERROR: Parsed unknown light source type!" << std::flush << std::endl; exit(EXIT_FAILURE);
				break;
			}
		}

		// finally, read the camera setup
		in >> x >> y >> z; glm::vec3 pos(x,y,z);
		in >> x >> y >> z; glm::vec3 at(x,y,z);
		in >> x >> y >> z; glm::vec3 up(x,y,z);
		in >> fov;
		cam = new Camera(pos, at, up, fov);

		std::cout << " done" << std::endl;
	} else {
		std::cout << " failed" << std::endl;
		return false;
	}

	in.close();
	return true;
}

void SceneDescription::dumpScene() {
	std::list<Object*>::iterator it;
	std::list<Lightsource*>::iterator lit;

	std::cout << "====== SCENE DESCRIPTION ======" << std::endl;

	std::cout << "-- CAMERA:" << std::endl;
	std::cout << "Pos:"; PRVEC(cam->getPos());
	std::cout << ", Dir:"; PRVEC(cam->getDir());
	std::cout << ", Up:"; PRVEC(cam->getUp()); std::cout << std::endl;

	std::cout << "-- LIGHTS (" << lights.size() << "):" << std::endl;
	for(lit = lights.begin(); lit != lights.end(); ++lit)
		(*lit)->print();

	std::cout << "-- OBJECTS (" << objects.size() << "), BBox :";
	PRVEC(minPoint); std::cout << ","; PRVEC(maxPoint); std::cout << std::endl;
	if(objects.size() < 50) {
		for(it = objects.begin(); it != objects.end(); ++it)
			(*it)->print(std::cout);
	}
}

void SceneDescription::buildKdTree() {
	kdtree = new KdTree(this);
}

KdTree* SceneDescription::getKdTree() {
	return kdtree;
}

bool SceneDescription::loadMeshOFF(std::string fname, std::list<Object*>& objs, glm::vec3 meshCol, int material) {
	std::fstream in(fname.c_str());
	std::string off("OFF");
	char buf[OFF_READER_BUF];

	int numVertices, numTriangles, numEdges;

	if(in) {
        in.getline(buf, OFF_READER_BUF);
        if(off.compare(0, 3, buf, 3) != 0) return false;

        in >> numVertices >> numTriangles >> numEdges;
        vertices.resize(numVertices);
        vertexNormals.resize(numVertices, glm::vec3(0.0f));

        unsigned numVerticesPerFace, v1, v2, v3;
        float x, y, z;

        // sweep to get all vertex coords
        for(int i = 0; i < numVertices; i++) {
        	in >> x >> y >> z;
        	minPoint.x = std::min(minPoint.x, x); maxPoint.x = std::max(maxPoint.x, x);
        	minPoint.y = std::min(minPoint.y, y); maxPoint.y = std::max(maxPoint.y, y);
        	minPoint.z = std::min(minPoint.z, z); maxPoint.z = std::max(maxPoint.z, z);
        	vertices[i] = glm::vec3(x,y,z);
        }

        // sweep to create triangles from vertex array
        for(int i = 0; i < numTriangles; i++) {
        	in >> numVerticesPerFace;
        	if(numVerticesPerFace != 3) return false;
        	in >> v1 >> v2 >> v3;
        	Object* tri = new Triangle(&vertices[v1], &vertices[v2], &vertices[v3], // vertex pointers
									   &vertexNormals[v1], &vertexNormals[v2], &vertexNormals[v3], // vertex normal pointers
									   meshCol, // color
									   globalObjID++); // obj id
        	objs.push_back(tri);
        }

        // sweep to normalize per-vertex normals
        for(int i = 0; i < numVertices; i++) {
        	vertexNormals[i] = glm::normalize(vertexNormals[i]);
        }
	} else {
		return false;
	}

	in.close();
	return true;
}



