#pragma once
#include "ofxOpenCv.h"

class EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) {};
	virtual double evaluate(cv::Mat im) { 
		return 0; 
	};
};
