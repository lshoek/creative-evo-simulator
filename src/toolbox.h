#pragma once
#include "ofMain.h"

class tb 
{
public:
	struct bounds
	{
		bounds() { min = 0; max = 1.0f; };
		bounds(float minimum, float maximum)
		{
			min = minimum;
			max = maximum;
		}
		float min;
		float max;
	};

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
