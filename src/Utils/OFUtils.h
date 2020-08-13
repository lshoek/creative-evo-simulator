#pragma once
#include "ofVectorMath.h"
#include "ofEvent.h"
#include "ofLog.h"

class OFUtils
{
public:
	static float inv(float v)
	{
		return 1.0f - v;
	}

	static int swap(int i)
	{
		return (i + 1) % 2;
	}

	static glm::vec3 getMouse()
	{
		return glm::vec3(ofGetMouseX(), ofGetMouseY(), 0);
	}
};
