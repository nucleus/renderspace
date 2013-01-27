/*
 * MemManager.hpp
 *
 *  Created on: 21.01.2013
 *      Author: michael
 */

#ifndef MEMMANAGER_HPP_
#define MEMMANAGER_HPP_

#include <list>

#include "KdTree.hpp"

class KdTreeNode;
class Object;

#define DEFAULT_KDTREE_NODES_ALLOC 1000000
#define DEFAULT_OBJLIST_PTR_ALLOC 100000

class MemManager {
public:
	MemManager();
	KdTreeNode* allocateKdTreeNodePair();
	std::list<Object*>* allocateObjListPtr();
private:
	std::list<Object*>* m_OList;
	char* m_KdArray, *m_ObjArray;
	KdTreeNode* m_KdPtr;
	std::list<Object*>* m_ObjPtr;
};

#endif /* MEMMANAGER_HPP_ */
