#pragma once
#include "SimNodeBase.h"

class SimCreature;

class SimNode : public SimNodeBase
{
public:
	SimNode(int tag, btDynamicsWorld* owner);
	SimNode(int tag, ofColor color, btDynamicsWorld* owner);
	virtual ~SimNode();

	// conveniece init functions
	void initBox(btVector3 position, btVector3 size, float mass);
	void initCapsule(btVector3 position, float radius, float height, float mass);
	void initPlane(btVector3 position, float size, float mass);

	virtual void draw() override;
	virtual void drawImmediate() override;

	SimCreature* getCreaturePtr();
	void setCreatureOwner(SimCreature* creaturePtr);

	virtual void setColor(ofColor color) override;
	void setInkColor(ofColor inkColor);

	bool isBrush();
	bool isBrushActivated();
	float getBrushPressure();
	void setBrushPressure(float pressure);

private:
	SimCreature* _ownerCreature = nullptr;

	ofColor _cachedColor = ofColor::black;
	ofColor _inkColor = ofColor::black;
	float _brushMinThreshold = 0.5f;
	float _brushPressure = 0.0f;
};
