#pragma once
#include "Artifact/ofxFractalCompression/ofxFractalCompression.h"
#include "ofxOpenCv.h"

class Evaluator
{
public:
	void setup();
	double evaluate(cv::Mat im);

private:
	ofxFractalCompression _compressor;
};
