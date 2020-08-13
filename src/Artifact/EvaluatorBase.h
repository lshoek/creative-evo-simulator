#pragma once
#include "ofxOpenCv.h"

class EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) = 0;
	virtual std::vector<double> evaluate(cv::Mat im) = 0;
};
