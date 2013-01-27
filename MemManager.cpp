/*
 * MemManager.cpp
 *
 *  Created on: 21.01.2013
 *      Author: michael
 */

#include "MemManager.hpp"

MemManager::MemManager() {
	m_KdArray = (char*)(new KdTreeNode[DEFAULT_KDTREE_NODES_ALLOC]);
	m_ObjArray = (char*)(new std::list<Object*>[DEFAULT_OBJLIST_PTR_ALLOC]);

	unsigned long addr = (unsigned long)m_KdArray;
	m_KdPtr = (KdTreeNode*)((addr + 32) & (0xffffffff - 31));

	addr = (unsigned long)m_ObjArray;
	m_ObjPtr = (std::list<Object*>*)((addr + 32) & (0xffffffff - 31));
}

std::list<Object*>* MemManager::allocateObjListPtr() {
	return m_ObjPtr++;
}

KdTreeNode* MemManager::allocateKdTreeNodePair() {
	unsigned long* tmp = (unsigned long*)m_KdPtr;
	tmp[1] = tmp[3] = 6;
	KdTreeNode* node = m_KdPtr;
	m_KdPtr += 2;
	return node;
}
