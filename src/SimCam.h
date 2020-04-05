#pragma once
#include "ofMain.h"
#include "SimNode.h"

class SimCam : public ofCamera
{
public:
	void setup();
	void update();
	void setFocusNode(SimNode* node);

	void mouseMoved(ofMouseEventArgs& mouse);
	void mouseDragged(ofMouseEventArgs& mouse);

	SimNode* focusNodePtr = nullptr;

	glm::vec3 forward;

	float minDistToNode = 2.0f;
	float mouseDamping = 0.25f;

	glm::vec3 curMouse;
	glm::vec3 prevMouse;

	glm::vec3 diff;
	glm::vec3 rawDiff;
	float maxDiffLength = 50.0f;

	bool bHasFocusNode = false;

	bool bLMouseDown = false;
	bool bLMouseDragged = false;
	bool bRMouseDown = false;
	bool bRMouseDragged = false;
};
