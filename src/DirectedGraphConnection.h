#pragma once
#include "bullet/btBulletCollisionCommon.h"

class GraphNode;

class GraphConnection
{
public:
	struct JointInfo {
		btVector3 parentAnchorDir = btVector3(1, 0, 0);
		btVector3 childAnchorDir = btVector3(-1, 0, 0);
		btQuaternion rotation = btQuaternion::getIdentity();
		btScalar scalingFactor = 1.0;
		btVector3 axis = btVector3(1, 0, 0);
		btVector3 reflection = btVector3(0, 0, 0);

		JointInfo() {}
		JointInfo(btVector3 anchorDir, btQuaternion rot, btScalar scale, btVector3 axis, btVector3 reflection) :
			parentAnchorDir(anchorDir), rotation(rot), scalingFactor(scale), axis(axis), reflection(reflection) {}
	};
	GraphConnection(GraphNode* from, GraphNode* to, JointInfo info, bool bIsTerminal);
	GraphConnection(GraphNode* from, GraphNode* to, bool bIsTerminal);

	JointInfo jointInfo;

	GraphNode* parent;
	GraphNode* child;

	bool bIsRecurrent = false;
	bool bIsTerminalFlag = false;
};
