/*
 * KdTree.hpp
 *
 *  Created on: 14.01.2013
 *      Author: michael
 */

#ifndef KDTREE_HPP_
#define KDTREE_HPP_

#include <list>
#include <string>

#include "SceneDescription.hpp"
#include "Ray.hpp"
#include "MemManager.hpp"

#define START_AXIS 0
#define NODE_MIN_OBJS 5
#define TREE_DEPTH_MAX 20
#define KDTREE_STACK_DEPTH 50

#define USE_KDTREE 1
#define KDTREE_LARGENODES 0

class Object;
class AABB;
class Sphere;
class Plane;
class SceneDescription;
class MemManager;

/*
 * A k-d tree node. Not optimized for memory, currently takes
 * 1 + 4 + 4 + 4 + 4 + 1 = 18 bytes.
 */
#if KDTREE_LARGENODES == 1
class KdTreeNode {
public:
	KdTreeNode();
	KdTreeNode(std::list<Object*>* list);

	void setAxis(char axis) { this->axis = axis; }
	char getAxis() { return axis; }
	void setSplitPos(float pos) { splitPos = pos; }
	float getSplitPos() { return splitPos; }
	void setLeft(KdTreeNode* node) { leftChild = node; }
	KdTreeNode* getLeft() { return leftChild; }
	void setRight(KdTreeNode* node) { rightChild = node; }
	KdTreeNode* getRight() { return rightChild; }
	bool isLeaf() { return leaf; }
	void setLeaf(bool leaf) { this->leaf = leaf; }
	void setObjectList(std::list<Object*>* objList) { this->objList = objList; }
	std::list<Object*>* getObjectList() { return objList; }
private:
	char axis;
	float splitPos;
	bool leaf;
	KdTreeNode* leftChild, *rightChild;
	std::list<Object*>* objList;
};

#else

/*
 * A k-d tree node. Heavily optimized for benign cache behavior.
 * Takes 4+4 = 8 bytes in memory, i.e. 4/8 nodes fit in a single
 * L1 cache line.
 */
class KdTreeNode {
public:
	KdTreeNode();

	void setAxis(char axis) { data = (data & 0xfffffffc) + axis; }
	char getAxis() { return (char)(data & 3); }
	void setSplitPos(float pos) { split = pos; }
	float getSplitPos() { return split; }
	void setLeft(KdTreeNode* left) { data = (unsigned long)left + (data & 7); }
	KdTreeNode* getLeft() { return (KdTreeNode*)(data&0xfffffff8); }
	KdTreeNode* getRight() { return ((KdTreeNode*)(data&0xfffffff8)) + 1; }
	bool isLeaf() { return ((data & 4) > 0); }
	void setLeaf(bool leaf) { data = (leaf)?(data|4):(data&0xfffffffb); }
	std::list<Object*>* getObjectList() {
		if(!isLeaf()) {
			std::cerr << "ERROR: Trying to grab objects from non-leaf kdtree node" << std::endl;
			return NULL;
		}
		return objList;
	}

	void setObjectList(std::list<Object*>* list) {
		if(!isLeaf()) {
			std::cerr << "ERROR: Trying to set objects on non-leaf kdtree node" << std::endl;
			return;
		}
		objList = list;
	}
private:
	union {
		float split;
		std::list<Object*>* objList;
	};
	unsigned data;
};

#endif

/*
 * A stack entry for accelerated non-recursive traversal. See
 * PhD thesis of Vlastimil Havran, Appendix C.
 */
struct StackElem {
	KdTreeNode* node; 	/* pointer to far child */
	float t;			/* entry/exit signed distance */
	glm::vec3 pb;		/* coordinate of entry/exit point */
	int prev;			/* pointer to previous stack item */
};

/*
 * Split candidate list for the SAH kdtree construction implementation.
 */
struct SplitList {
	float pos;
	int n1count, n2count;
	SplitList* next;
};

/*
 * The full kd-tree constructed from scene geometry.
 */
class KdTree {
public:
	KdTree(SceneDescription* scene);
	~KdTree();

	/*
	 * Traverse the kd-tree and test the ray for object intersection. If it intersects,
	 * the hit object and intersection distance are returned.
	 */
	bool traverseAndIntersect(Ray& ray, Object* &objIntersect, float& tIntersect, unsigned long long excludeID);

	/*
	 * Similar to above, but for lights.
	 */
	bool traverseAndFindLightOccluders(glm::vec3 start, glm::vec3 target);

	/*
	 * Prints the current contents of the tree.
	 */
	void print();
private:
	KdTreeNode* root;
	MemManager* manager;
	glm::vec3 min, max;
	AABB* treeBBox;
	char mod[5];
	SplitList* sPool, *sList;

#if KDTREE_LARGENODES == 1
	void build(KdTreeNode* node, int depth = 0);
#else
	void build(KdTreeNode* node, AABB& bbox, std::list<Object*>* list, int depth = 0);
	void buildSAH(KdTreeNode* node, std::list<Object*>* list, int depth = 0);
	void subdivide(KdTreeNode* node, std::list<Object*>* list, AABB& box, int depth, int prims);
	void insertSplitPos(float pos);
#endif
	void destroy(KdTreeNode* node);
	float findOptimalSplitPosition(std::list<Object*>* list, char axis);
	void printNodes(KdTreeNode* node, std::string indent);
};

#endif /* KDTREE_HPP_ */
