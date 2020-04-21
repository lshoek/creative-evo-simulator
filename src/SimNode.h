#pragma once
#include "SimNodeBase.h"

class SimNode : public SimNodeBase
{
public:
	SimNode(int tag, btDynamicsWorld* owner);
	SimNode(int tag, ofColor color, btDynamicsWorld* owner);
	~SimNode();

	// conveniece init functions
	void initBox(glm::vec3 position, glm::vec3 size, float mass);
	void initCapsule(glm::vec3 position, float radius, float height, float mass);
	void initPlane(glm::vec3 position, float size, float mass);

	virtual void draw() override;
};
