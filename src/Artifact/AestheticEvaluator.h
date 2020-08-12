#pragma once
#include "Artifact/EvaluatorBase.h"
#include "Artifact/FractalCompressor/FractalCompressor.h"
#include "ofxOpenCv.h"

class AestheticEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override;
	virtual double evaluate(cv::Mat im) override;

	void setDecodingDepth(int depth);

private:

	// Remaps coverage to a biased curve [0..1]
	double coverageFunc(double coverage);

	FractalCompressor _compressor;

	int _decodingDepth = 3;
	double _maxCoverage;

	double pc0LowerBound = 1.0;
	double pc1Lowerbound = 1.0;
};
