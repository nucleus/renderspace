/*
 * KdTree.cpp
 *
 *  Created on: 14.01.2013
 *      Author: michael
 */

#include <algorithm>
#include <vector>
#include <iostream>

#include "KdTree.hpp"
#include "glm.hpp"
#include "gtx/fast_square_root.hpp"

/* /////////////////////////////////////////////////
 * //////////////////// TREENODE ///////////////////
 * ///////////////////////////////////////////////*/
#if KDTREE_LARGENODES == 1
KdTreeNode::KdTreeNode() {
	this->axis = 0;
	this->leaf = false;
	this->splitPos = 0.0f;
	this->leftChild = NULL;
	this->rightChild = NULL;
	this->objList = new std::list<Object*>();
}

KdTreeNode::KdTreeNode(std::list<Object*>* list) {
	this->axis = 0;
	this->leaf = false;
	this->splitPos = 0.0f;
	this->leftChild = NULL;
	this->rightChild = NULL;
	this->objList = list;
}

#else

KdTreeNode::KdTreeNode() {
	setAxis(0);
	setLeaf(false);
	setSplitPos(0.0f);
	setLeft(NULL);

	if(sizeof(std::list<Object*>*) != sizeof(float)) {
		std::cerr << "WARNING: KdTree node size will exceed 8 bytes, this causes bad performance" << std::endl;
	}
}
#endif
/* /////////////////////////////////////////////////
 * //////////////////// TREE ///////////////////////
 * ///////////////////////////////////////////////*/

#if KDTREE_LARGENODES == 1
KdTree::KdTree(SceneDescription* scene) {
	root = new KdTreeNode();
	root->setLeaf(false);
	root->setAxis(START_AXIS);
	root->setObjectList(&(scene->getObjects()));

	this->min = scene->getMinPoint();
	this->max = scene->getMaxPoint();
	treeBBox = new AABB(min, max);

	mod[0] = 0; mod[1] = 1; mod[2] = 2; mod[3] = 0; mod[4] = 1;

	this->manager = new MemManager();

	std::cout << "Constructing Kd-tree, node size in memory is " << sizeof(KdTreeNode) << " bytes ... " << std::flush;
	build(root);
	std::cout << "done" << std::endl;
}
#else
KdTree::KdTree(SceneDescription* scene) {
	root = new KdTreeNode();
	root->setLeaf(false);
	root->setAxis(START_AXIS);

	this->min = scene->getMinPoint();
	this->max = scene->getMaxPoint();
	treeBBox = new AABB(min, max);

	mod[0] = 0; mod[1] = 1; mod[2] = 2; mod[3] = 0; mod[4] = 1;

	this->manager = new MemManager();

	std::cout << "Constructing Kd-tree from " << scene->getObjects().size() << " objects, node size in memory is " << sizeof(KdTreeNode) << " bytes ... " << std::flush;
//	buildSAH(root, &(scene->getObjects()));
	build(root, *treeBBox, &(scene->getObjects()));
	std::cout << "done" << std::endl;
}
#endif

KdTree::~KdTree() {
	// TODO: something is wrong here (valgrind)
	destroy(root);
	delete treeBBox;
	delete manager;
}

#if KDTREE_LARGENODES == 1
void KdTree::build(KdTreeNode *node, int depth) {
	std::list<Object*>* list = node->getObjectList();

	// if max depth reached of only few objs left in node, set to leaf and break rec.
	if(depth > TREE_DEPTH_MAX || list->size() < NODE_MIN_OBJS) {
		node->setLeaf(true);
		return;
	}

	// find splitting position (and axis?)
	char nextAxis = mod[node->getAxis() + 1];
	float splitPos = findOptimalSplitPosition(list, node->getAxis());
	node->setSplitPos(splitPos);

	// create the children
	KdTreeNode* leftChild = new KdTreeNode();
	leftChild->setLeaf(false);
	leftChild->setAxis(nextAxis);
	node->setLeft(leftChild);
	KdTreeNode* rightChild = new KdTreeNode();
	rightChild->setLeaf(false);
	rightChild->setAxis(nextAxis);
	node->setRight(rightChild);

	// distribute the objects to the children
	std::list<Object*>::iterator it = list->begin();
	for( ; it != list->end(); ++it) {
		glm::vec2 bounds = (*it)->getMinMaxAlongDimension(node->getAxis());
		if(bounds[0] < splitPos) leftChild->addObject((*it));
		if(bounds[1] > splitPos) rightChild->addObject((*it));
	}
	// be tidy
	if(node != root) {
		delete list;
		node->setObjectList(NULL);
	}
	// recurse
	build(leftChild, depth+1);
	build(rightChild, depth+1);
}
#else
void KdTree::build(KdTreeNode* node, AABB& bbox, std::list<Object*>* list, int depth) {
	// if max depth reached of only few objs left in node, set to leaf and break rec.
	if(depth > TREE_DEPTH_MAX || list->size() < NODE_MIN_OBJS) {
		node->setLeaf(true);
		node->setObjectList(list);
		return;
	}

	// find splitting position (and axis?)
//	float splitPos = findOptimalSplitPosition(list, node->getAxis());
	if(bbox.extents.y >= bbox.extents.x && bbox.extents.y > bbox.extents.z)
		node->setAxis(1);
	else if(bbox.extents.z >= bbox.extents.x && bbox.extents.z > bbox.extents.y)
		node->setAxis(2);
	char axis = node->getAxis();
	float splitPos = (bbox.minCorner[axis] + bbox.maxCorner[axis]) / 2.0f;
	node->setSplitPos(splitPos);
	node->setLeaf(false);

	// create the children
	std::list<Object*>* leftList = new std::list<Object*>();
	std::list<Object*>* rightList = new std::list<Object*>();

	AABB leftBox(bbox.minCorner, bbox.maxCorner);
	leftBox.maxCorner[axis] = splitPos;
	leftBox.extents[axis] = leftBox.maxCorner[axis] - leftBox.minCorner[axis];
	leftBox.center[axis] = leftBox.minCorner[axis] + 0.5f * leftBox.extents[axis];

	AABB rightBox(bbox.minCorner, bbox.maxCorner);
	rightBox.minCorner[axis] = splitPos;
	rightBox.extents[axis] = rightBox.maxCorner[axis] - rightBox.minCorner[axis];
	rightBox.center[axis] = rightBox.minCorner[axis] + 0.5f * rightBox.extents[axis];

	char nextAxis = mod[node->getAxis() + 1];
	KdTreeNode* leftChild = manager->allocateKdTreeNodePair();
	leftChild->setLeaf(false);
	leftChild->setAxis(nextAxis);
	KdTreeNode* rightChild = leftChild + 1;
	rightChild->setLeaf(false);
	rightChild->setAxis(nextAxis);

	// distribute the objects to the children
	std::list<Object*>::iterator it = list->begin();
	for( ; it != list->end(); ++it) {
		glm::vec2 bounds = (*it)->getMinMaxAlongDimension(node->getAxis());
		if(bounds[0] < splitPos) leftList->push_back((*it));
		if(bounds[1] >= splitPos) rightList->push_back((*it));
	}
	// be tidy
	if(node != root)
		delete list;

	node->setLeft(leftChild);

	// recurse
	build(leftChild, leftBox, leftList, depth+1);
	build(rightChild, rightBox, rightList, depth+1);

}

void KdTree::buildSAH(KdTreeNode* node, std::list<Object*>* list, int depth) {
	int prims = list->size(), i;
	sPool = new SplitList[prims * 2 + 8];
	for(i = 0; i < (prims * 2 + 6); i++) sPool[i].next = &sPool[i+1];
	sPool[i].next = 0;
	sList = 0;
	subdivide(node, list, *treeBBox, depth, prims);
}

void KdTree::subdivide(KdTreeNode* node, std::list<Object*>* list, AABB& box, int depth, int prims) {
	if(sList) {
		SplitList* list = sList;
		while(list->next) list = list->next;
		list->next = sPool;
		sPool = sList; sList = 0;
	}

	// determine split axis
	glm::vec3 ex = box.extents;
	if(ex.x >= ex.y && ex.x >= ex.z) node->setAxis(0);
	else if(ex.y >= ex.x && ex.y >= ex.z) node->setAxis(1);
	int axis = node->getAxis();

	// make a list of the split position candidates
	float p1, p2;
	float pos1 = box.minCorner[axis];
	float pos2 = box.maxCorner[axis];

	bool* pright = new bool[prims];
	float* eleft = new float[prims], *eright = new float[prims];
	Object** objarray = new Object*[prims];
	int aidx = 0;
	std::list<Object*>::iterator it = list->begin();
	while(it != list->end()) {
		Object* obj = objarray[aidx] = (*it);
		pright[aidx] = true;
		glm::vec2 dims = obj->getMinMaxAlongDimension(axis);
		eleft[aidx] = dims.x;
		eright[aidx] = dims.y;
		aidx++;
		switch(obj->type) {
		case OBJ_SPHERE:
			p1 = ((Sphere*)obj)->pos[axis] - ((Sphere*)obj)->rad;
			p2 = ((Sphere*)obj)->pos[axis] + ((Sphere*)obj)->rad;
			if(p1 >= pos1 && p1 <= pos2) insertSplitPos(p1);
			if(p2 >= pos1 && p2 <= pos2) insertSplitPos(p2);
			break;
		case OBJ_AABB:
			p1 = ((AABB*)obj)->minCorner[axis] - 0.001f;
			p2 = ((AABB*)obj)->maxCorner[axis] + 0.001f;
			if(p1 >= pos1 && p1 <= pos2) insertSplitPos(p1);
			if(p2 >= pos1 && p2 <= pos2) insertSplitPos(p2);
			break;
		case OBJ_PLANE:
			p1 = ((Plane*)obj)->anchor[axis] - 0.001f;
			if(p1 >= pos1 && p1 <= pos2) insertSplitPos(p1);
			break;
		case OBJ_TRIANGLE:
			for(int i = 0; i < 3; i++) {
				p1 = ((Triangle*)obj)->vertices[i][0][axis];
				if((p1 >= pos1) && (p1 <= pos2)) insertSplitPos(p1);
			}
			break;
		}
		it++;
	}

	// determine n1count / n2count for each split position
	AABB b1(box.minCorner, box.maxCorner), b2(box.minCorner, box.maxCorner), b3(box.minCorner, box.maxCorner), b4(box.minCorner, box.maxCorner);
	SplitList* splist = sList;
	float b3p1 = b3.minCorner[axis];
	float b4p2 = b4.maxCorner[axis];
	while(splist) {
		b4.minCorner[axis] = splist->pos;
		b4.maxCorner[axis] = b4.minCorner[axis] + pos2 - splist->pos;
		b4.extents[axis] = b4.maxCorner[axis] - b4.minCorner[axis];
		b4.center[axis] = 0.5f * (b4.minCorner[axis] + b4.maxCorner[axis]);

		b3.maxCorner[axis] = b3.minCorner[axis] + splist->pos - pos1;
		b3.extents[axis] = b3.maxCorner[axis] - b3.minCorner[axis];
		b3.center[axis] = 0.5f * (b3.minCorner[axis] + b3.maxCorner[axis]);

		float b3p2 = b3.maxCorner[axis];
		float b4p1 = b4.minCorner[axis];
		for(int i = 0; i < prims; i++) if(pright[i]) {
			Object* obj = objarray[i];
			if((eleft[i] <= b3p2) && (eright[i] >= b3p1)) {
				if(obj->intersect(b3))
					splist->n1count++;
			}
			if((eleft[i] <= b4p2) && (eright[i] >= b4p1)) {
				if(obj->intersect(b4))
					splist->n2count++;
				else
					pright[i] = false;
			}
		}
		else
			splist->n1count++;
		splist = splist->next;
	}
	delete pright;
	float SAV = 0.5f / (box.getw() * box.getd() + box.getw() * box.geth() + box.getd() * box.geth());
	float Cleaf = prims * 1.0f;
	splist = sList;
	float lowcost = 10000.0;
	float bestpos = 0;
	while(splist) {
		b4.minCorner[axis] = splist->pos;
		b4.extents[axis] = pos2 - splist->pos;
		b4.maxCorner[axis] = b4.minCorner[axis] + b4.extents[axis];
		b4.center[axis] = b4.minCorner[axis] + 0.5f * b4.extents[axis];

		b3.extents[axis] = splist->pos - pos1;
		b3.maxCorner[axis] = b3.minCorner[axis] + b3.extents[axis];
		b3.center[axis] = b3.minCorner[axis] + 0.5f * b3.extents[axis];

		float SA1 = 2.0 * (b3.getw() * b3.getd() + b3.getw() * b3.geth() + b3.getd() * b3.geth());
		float SA2 = 2.0 * (b4.getw() * b4.getd() + b4.getw() * b4.geth() + b4.getd() * b4.geth());
		float splitcost = 0.3f + 1.0f * (SA1 * SAV * splist->n1count + SA2 * SAV * splist->n2count);

		if(splitcost < lowcost) {
			lowcost = splitcost;
			bestpos = splist->pos;
			b1 = b3, b2 = b4;
		}
		splist = splist->next;
	}

	if(lowcost > Cleaf) {
		node->setLeaf(true);
		node->setObjectList(list);
		return;
	}

	node->setSplitPos(bestpos);
	KdTreeNode* left = manager->allocateKdTreeNodePair();
	KdTreeNode* right = left + 1;
	left->setLeaf(false);
	right->setLeaf(false);
	std::list<Object*>* leftList = new std::list<Object*>();
	std::list<Object*>* rightList = new std::list<Object*>();
	int n1count = 0, n2count = 0, total = 0;

	float b1p1 = b1.minCorner[axis];
	float b2p2 = b2.maxCorner[axis];
	float b1p2 = b1.maxCorner[axis];
	float b2p1 = b2.minCorner[axis];

	for(int i = 0; i < prims; i++) {
		Object* obj = objarray[i];
		total++;
		if((eleft[i] <= b1p2) && (eright[i] >= b1p1)) if(obj->intersect(b1)) {
			leftList->push_back(obj);
			n1count++;
		}
		if((eleft[i] <= b2p2) && (eright[i] >= b2p1)) if(obj->intersect(b2)) {
			rightList->push_back(obj);
			n2count++;
		}
	}
	delete eleft;
	delete eright;
	delete objarray;
	if(node != root)
		delete list;
	node->setLeft(left);
	node->setLeaf(false);

	if(depth < TREE_DEPTH_MAX) {
		if(n1count > 2)
			subdivide(left, leftList, b1, depth + 1, n1count);
		else {
			left->setLeaf(true);
			left->setObjectList(leftList);
		}

		if(n2count > 2)
			subdivide(left+1, rightList, b2, depth + 1, n2count);
		else {
			right->setLeaf(true);
			right->setObjectList(rightList);
		}
	} else {
		left->setLeaf(true);
		left->setObjectList(leftList);
		right->setLeaf(true);
		right->setObjectList(rightList);
	}
}

void KdTree::insertSplitPos(float pos) {
	SplitList* entry = sPool;
	sPool = sPool->next;
	entry->next = 0;
	entry->pos = pos;
	entry->n1count = 0;
	entry->n2count = 0;
	if(!sList)
		sList = entry;
	else {
		if(pos < sList->pos) {
			entry->next = sList;
			sList = entry;
		} else if(pos == sList->pos) {
			entry->next = sPool;
			sPool = entry;
		} else {
			SplitList* list = sList;
			while((list->next) && (pos >= list->next->pos)) {
				if(pos == list->next->pos) {
					entry->next = sPool;
					sPool = entry;
					return;
				}
				list = list->next;
			}
			entry->next = list->next;
			list->next = entry;
		}
	}
}
#endif
bool KdTree::traverseAndIntersect(Ray& ray, Object* &objIntersect, float& tIntersect, unsigned long long excludeID) {
	glm::vec3 nearHit, farHit;
	float tnear = 0.0, tfar = (float)1e10, t;

	glm::vec3 p1 = min;
	glm::vec3 p2 = max;
	glm::vec3 D = ray.dir, O = ray.org;

	for(int i = 0; i < 3; i++)
		if (D[i] < 0) {
			if(O[i] < p1[i]) return false;
		} else if (O[i] > p2[i])
			return false;
	// clip ray segment to box
	for (int i = 0; i < 3; i++)	{
		float pos = O[i] + tfar * D[i];
		if (D[i] < 0) {
			// clip end point
			if (pos < p1[i]) tfar = tnear + (tfar - tnear) * ((O[i] - p1[i]) / (O[i] - pos));
			// clip start point
			if (O[i] > p2[i]) tnear += (tfar - tnear) * ((O[i] - p2[i]) / (tfar * D[i]));
		} else {
			// clip end point
			if (pos > p2[i]) tfar = tnear + (tfar - tnear) * ((p2[i] - O[i]) / (pos - O[i]));
			// clip start point
			if (O[i] < p1[i]) tnear += (tfar - tnear) * ((p1[i] - O[i]) / (tfar * D[i]));
		}
		if (tnear > tfar) return false;
	}

	StackElem stack[KDTREE_STACK_DEPTH];

	KdTreeNode* farChild, *currNode = root;
	int enPt = 0;
	stack[enPt].t = tnear;
	if(tnear > 0.0f)
		stack[enPt].pb = O + tnear * D;
	else
		stack[enPt].pb = O;

	int exPt = 1;
	stack[exPt].t = tfar;
	stack[exPt].pb = O + tfar * D;
	stack[exPt].node = NULL;

	while(currNode != NULL) {
		while(!currNode->isLeaf()) {
			float splitVal = currNode->getSplitPos();
			char axis = currNode->getAxis();
			char nextAxis = this->mod[axis+1];
			char prevAxis = this->mod[axis+2];
			if(stack[enPt].pb[axis] <= splitVal) {
				if(stack[exPt].pb[axis] <= splitVal) {
					currNode = currNode->getLeft();
					continue;
				}
				if(stack[exPt].pb[axis] == splitVal) {
					currNode = currNode->getRight();
					continue;
				}
				farChild = currNode->getRight();
				currNode = currNode->getLeft();
			} else {
				if(splitVal < stack[exPt].pb[axis]) {
					currNode = currNode->getRight();
					continue;
				}
				farChild = currNode->getLeft();
				currNode = currNode->getRight();
			}

			t = (splitVal - ray.org[axis]) / ray.dir[axis];

			int tmp = exPt;
			exPt++;
			if(exPt == enPt)
				exPt++;
			stack[exPt].prev = tmp;
			stack[exPt].t = t;
			stack[exPt].node = farChild;
			stack[exPt].pb[axis] = splitVal;
			stack[exPt].pb[nextAxis] = ray.org[nextAxis] + t * ray.dir[nextAxis];
			stack[exPt].pb[prevAxis] = ray.org[prevAxis] + t * ray.dir[prevAxis];
		}

		float ttest, maxt = (float)1e10;
		bool validIsectionFound = false, isectionFound;

		int threadID = omp_get_thread_num();
		for(std::list<Object*>::iterator it = currNode->getObjectList()->begin(); it != currNode->getObjectList()->end(); ++it) {
			// if object was visited before, ray id will be cached in a field in the object (fields are thread-private)
			if((*it)->rayIDs[threadID] == ray.rayID) {
				ttest = (*it)->intersectionTestResults[threadID];
				isectionFound = true;
			} else {
				// if object is visited for the first time, compute intersection point
				isectionFound = (*it)->intersect(ray, ttest);
			}

			// if an intersection exists (old or new), test intersection point against voxel bounds and previous known maximum
			if(isectionFound && (*it)->id != excludeID) {
				// if valid, i.e. in voxel and closer than current closest intersection, store and return
				if(ttest > stack[enPt].t && ttest < stack[exPt].t) {
					if(ttest < maxt) {
						maxt = ttest;

						validIsectionFound = true;
						objIntersect = (*it);
						tIntersect = ttest;
					}
				} else {
					// if it does not pass the voxel tests, store it for later
					(*it)->intersectionTestResults[threadID] = ttest;
					(*it)->rayIDs[threadID] = ray.rayID;
				}
			}
		}

		if(validIsectionFound)
			return true;

		enPt = exPt;
		currNode = stack[exPt].node;
		exPt = stack[enPt].prev;
	}

	return false;
}

bool KdTree::traverseAndFindLightOccluders(glm::vec3 start, glm::vec3 target) {
	glm::vec3 dir = glm::fastNormalize(target-start);
	Ray ray(start + 0.001f * dir, dir);

	float tnear = 0.0, tfar = (float)1e10, t;

	glm::vec3 p1 = min;
	glm::vec3 p2 = max;
	glm::vec3 D = ray.dir, O = ray.org;

	for(int i = 0; i < 3; i++)
		if (D[i] < 0) {
			if(O[i] < p1[i]) return true;
		} else if (O[i] > p2[i])
			return true;
	// clip ray segment to box
	for (int i = 0; i < 3; i++)	{
		float pos = O[i] + tfar * D[i];
		if (D[i] < 0) {
			// clip end point
			if (pos < p1[i]) tfar = tnear + (tfar - tnear) * ((O[i] - p1[i]) / (O[i] - pos));
			// clip start point
			if (O[i] > p2[i]) tnear += (tfar - tnear) * ((O[i] - p2[i]) / (tfar * D[i]));
		} else {
			// clip end point
			if (pos > p2[i]) tfar = tnear + (tfar - tnear) * ((p2[i] - O[i]) / (pos - O[i]));
			// clip start point
			if (O[i] < p1[i]) tnear += (tfar - tnear) * ((p1[i] - O[i]) / (tfar * D[i]));
		}
		if (tnear > tfar) return true;
	}

	StackElem stack[KDTREE_STACK_DEPTH];

	KdTreeNode* farChild, *currNode = root;
	int enPt = 0;
	stack[enPt].t = tnear;

	if(tnear > 0.0f)
		stack[enPt].pb = O + tnear * D;
	else
		stack[enPt].pb = O;

	int exPt  =1;
	stack[exPt].t = tfar;
	stack[exPt].pb = O + tfar * D;
	stack[exPt].node = NULL;

	while(currNode != NULL) {
		while(!currNode->isLeaf()) {
			float splitVal = currNode->getSplitPos();
			char axis = currNode->getAxis();
			char nextAxis = this->mod[axis+1];
			char prevAxis = this->mod[axis+2];
			if(stack[enPt].pb[axis] <= splitVal) {
				if(stack[exPt].pb[axis] <= splitVal) {
					currNode = currNode->getLeft();
					continue;
				}
				if(stack[exPt].pb[axis] == splitVal) {
					currNode = currNode->getRight();
					continue;
				}
				farChild = currNode->getRight();
				currNode = currNode->getLeft();
			} else {
				if(splitVal < stack[exPt].pb[axis]) {
					currNode = currNode->getRight();
					continue;
				}
				farChild = currNode->getLeft();
				currNode = currNode->getRight();
			}

			t = (splitVal - ray.org[axis]) / ray.dir[axis];

			int tmp = exPt;
			exPt++;
			if(exPt == enPt)
				exPt++;
			stack[exPt].prev = tmp;
			stack[exPt].t = t;
			stack[exPt].node = farChild;
			stack[exPt].pb[axis] = splitVal;
			stack[exPt].pb[nextAxis] = ray.org[nextAxis] + t * ray.dir[nextAxis];
			stack[exPt].pb[prevAxis] = ray.org[prevAxis] + t * ray.dir[prevAxis];
		}

		glm::vec3 dummy;
		float ttest, maxt = ray.getTValueForRayPoint(target);
		bool intersected;

		int threadID = omp_get_thread_num();
		for(std::list<Object*>::iterator it = currNode->getObjectList()->begin(); it != currNode->getObjectList()->end(); ++it) {
			if((*it)->material.refrLightContrib != 0.0) continue;

			if((*it)->rayIDs[threadID] == ray.rayID) {
				ttest = (*it)->intersectionTestResults[threadID];
				intersected = true;
			} else {
				// if object is visited for the first time, compute intersection point
				intersected = (*it)->intersect(ray, ttest);
			}

			if(intersected) {
				if(ttest > stack[enPt].t && ttest < stack[exPt].t) {
					if(ttest < maxt)
						return false;
				} else {
					(*it)->intersectionTestResults[threadID] = ttest;
					(*it)->rayIDs[threadID] = ray.rayID;
				}
			}
		}

		enPt = exPt;
		currNode = stack[exPt].node;
		exPt = stack[enPt].prev;
	}

	return true;
}

void KdTree::print() {
	std::cout << "Kd-Tree Layout:" << std::endl;
	printNodes(root, "");
}
#if KDTREE_LARGENODES == 1
void KdTree::printNodes(KdTreeNode* node, std::string indent) {
	if(node == NULL)
		return;

	std::string dim;
	if(node->getAxis() == 0) dim = "X";
	else if(node->getAxis() == 1) dim = "Y";
	else dim = "Z";

	std::list<Object*>* objs = node->getObjectList();
	unsigned objCount = 0;
	if(objs != NULL && node != root)
		objCount = objs->size();

	std::cout << indent << "Node: " << dim << "-split at " << node->getSplitPos();
	std::cout << ", Leaf: " << (node->isLeaf() ? "Yes" : "Noo") << ", ObjCount:" << objCount << std::endl;

	printNodes(node->getLeft(), indent+"  ");
	printNodes(node->getRight(), indent+"  ");
}


void KdTree::destroy(KdTreeNode* node) {
	/* sweep down the tree to remove all object lists that are not inside leaf nodes */
	if(node == NULL)
		return;
	if(node->getObjectList() != NULL) {
		delete node->getObjectList();
		node->setObjectList(NULL);
	}
	destroy(node->getLeft());
	destroy(node->getRight());
	delete node;
}

#else

void KdTree::printNodes(KdTreeNode* node, std::string indent) {
	if(node->isLeaf()) {
		std::cout << indent << "LEAF: " << node->getObjectList()->size() << " Objects" << std::endl;
		return;
	}

	std::string dim;
	if(node->getAxis() == 0) dim = "X";
	else if(node->getAxis() == 1) dim = "Y";
	else dim = "Z";

	std::cout << indent << "Node: " << dim << "-split at " << node->getSplitPos() << std::endl;

	printNodes(node->getLeft(), indent+"  ");
	printNodes(node->getRight(), indent+"  ");
}

void KdTree::destroy(KdTreeNode* node) {
	/* sweep down the tree to remove all object lists that are not inside leaf nodes */
	if(node->isLeaf()) {
		delete node->getObjectList();
		return;
	}
	destroy(node->getLeft());
	destroy(node->getRight());
//	delete node;
}
#endif

float KdTree::findOptimalSplitPosition(std::list<Object*>* list, char axis) {
	std::list<Object*>::iterator it = list->begin();

	std::vector<float> positions;
	glm::vec3 pos;
	for( ; it != list->end(); ++it) {
		switch((*it)->type) {
		case OBJ_PLANE:
			positions.push_back(((Plane*)(*it))->anchor[axis]); break;
		case OBJ_SPHERE:
			positions.push_back(((Sphere*)(*it))->pos[axis] - ((Sphere*)(*it))->rad - 0.001f); break;
		case OBJ_AABB:
			positions.push_back(((AABB*)(*it))->minCorner[axis] - 0.001f); break;
		case OBJ_TRIANGLE:
			for(int i = 0; i < 3; i++) positions.push_back(((Triangle*)(*it))->vertices[i][0][axis]); break;
		}
	}

	// median (not optimal)
	std::sort(positions.begin(), positions.end());
	return positions[positions.size()/2];
}



