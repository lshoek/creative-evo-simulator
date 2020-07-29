#pragma once
#include "Artifact/EvaluatorBase.h"
#include "Artifact/ofxFractalCompression/ofxFractalCompression.h"
#include "ofxOpenCv.h"

class AestheticEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override;
	virtual double evaluate(cv::Mat im) override;

	void setDecodingDepth(int depth);

private:
	ofxFractalCompression _compressor;

	int _decodingDepth = 4;

	double minComplexity = 1.0;
	double maxComplexity = 10.0;

	double pc0LowerBound = 1.0;
	double pc1Lowerbound = 1.0;
	double icUpperBound = 20.0;
};
