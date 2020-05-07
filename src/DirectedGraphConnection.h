#pragma once
#include "bullet/btBulletCollisionCommon.h"

class GraphNode;

class GraphConnection
{
public:
	struct JointInfo {
		btVector3 childAnchorDir = btVector3(-1, 0, 0);
		btVector3 axis = btVector3(1, 0, 0);
		btScalar scalingFactor = 1.0;
		uint32_t fromIndex = -1;
		uint32_t toIndex = -1;

		JointInfo() {}
		JointInfo(btVector3 anchorDir, btScalar scale, btVector3 axis) :
			childAnchorDir(anchorDir), scalingFactor(scale), axis(axis) {}
	};
	GraphConnection(GraphNode* from, GraphNode* to, const JointInfo& info);

	JointInfo jointInfo;

	GraphNode* parent;
	GraphNode* child;

	bool bIsRecurrent = false;
	bool bIsTerminalFlag = false;
};
