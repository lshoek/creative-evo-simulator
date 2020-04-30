#pragma once
#include "bullet/btBulletCollisionCommon.h"

class GraphNode;

class GraphConnection
{
public:
	struct JointInfo {
		btVector3 parentAnchor = btVector3(1, 0, 0);
		btVector3 childAnchor = btVector3(1, 0, 0);
		btVector3 axis = btVector3(1, 0, 0);
		btQuaternion rotation = btQuaternion::getIdentity();
		btScalar scalingFactor = 1.0;
		btVector3 reflection = btVector3(0, 0, 0);

		JointInfo() {}
		JointInfo(btVector3 anchor, btQuaternion rot, btScalar scale, btVector3 reflection) :
			parentAnchor(anchor), rotation(rot), scalingFactor(scale), reflection(reflection) {}
	};
	GraphConnection(GraphNode* from, GraphNode* to, JointInfo info, bool bIsTerminal);
	GraphConnection(GraphNode* from, GraphNode* to, bool bIsTerminal);

	JointInfo jointInfo;

	GraphNode* parent;
	GraphNode* child;

	bool bIsRecurrent = false;
	bool bIsTerminalFlag = false;
};
