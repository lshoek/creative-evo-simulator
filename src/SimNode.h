#pragma once
#include "SimNodeBase.h"

class SimNode : public SimNodeBase
{
public:
	SimNode(int tag, btDynamicsWorld* owner);
	SimNode(int tag, ofColor color, btDynamicsWorld* owner);
	~SimNode();

	// conveniece init functions
	void initBox(btVector3 position, btVector3 size, float mass);
	void initCapsule(btVector3 position, float radius, float height, float mass);
	void initPlane(btVector3 position, float size, float mass);

	virtual void draw() override;
};
