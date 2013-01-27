/*
 * common.hpp
 *
 *  Created on: 10.01.2013
 *      Author: michael
 */

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include "glm.hpp"

#define ABS(a) ((a) < 0 ? -(a) : (a))
#define SGN(a) ((a) < 0 ? (-1) : (1))
#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) > (b) ? (b) : (a))
#define TRIMAX(a,b,c) ((a) < (b) ? ((b) < (c) ? (c) : (b)) : ((a) < (c) ? (c) : (a)))
#define TRIMIN(a,b,c) ((a) > (b) ? ((b) > (c) ? (c) : (b)) : ((a) > (c) ? (c) : (a)))
#define PRVEC(a) (std::cout << "(" << a.x << "," << a.y << "," << a.z << ")")

/* componentwise vector-vector multiplication */
inline glm::vec3 operator*(glm::vec3 a, glm::vec3 b) {
	return glm::vec3(a.x*b.x, a.y*b.y, a.z*b.z);
}

/* dim3 float vector scaling */
inline glm::vec3 operator*(glm::vec3 a, float s) {
	return glm::vec3(a.x*s, a.y*s, a.z*s);
}

/* dim3 float vector scaling */
inline glm::vec3 operator*(float s, glm::vec3 a) {
	return a*s;
}

/* dim3 float vector division */
inline glm::vec3 operator/(glm::vec3 a, float s) {
	return glm::vec3(a.x/s, a.y/s, a.z/s);
}

inline bool operator<(glm::vec3 a, glm::vec3 b) {
	return a.x < b.x && a.y < b.y && a.z < b.z;
}

inline bool operator>(glm::vec3 a, glm::vec3 b) {
	return a.x > b.x && a.y > b.y && a.z > b.z;
}

/* squared distance between two points in 3D space */
inline float dist(glm::vec3 a, glm::vec3 b) {
	return ((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.y-b.y)*(a.y-b.y));
//	return glm::length(a-b);
}

#endif /* COMMON_HPP_ */
