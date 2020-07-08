#pragma once
#include "Artifact/EvaluatorBase.h"
#include "Artifact/ofxFractalCompression/ofxFractalCompression.h"
#include "ofxOpenCv.h"

class AestheticEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override;
	virtual double evaluate(cv::Mat im) override;

private:
	ofxFractalCompression _compressor;
};
