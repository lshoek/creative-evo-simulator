#pragma once
#include "Artifact/EvaluatorBase.h"
#include "Artifact/FractalCompressor/FractalCompressor.h"
#include "ofxOpenCv.h"

class OrderlyCoverageEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override;
	virtual std::vector<double> evaluate(cv::Mat im) override;

	double getLatestCoverageScore();
	void setDecodingDepth(int depth);

private:
	double coverageFunc(double coverage);

	int _decodingDepth = 3;

	double _maxCoverageReward;
	double _latestCoverageScore;

	double minComplexity = 0.0025;
	double maxComplexity = 0.4;

	double pc0LowerBound = 1.0;
	double pc1Lowerbound = 1.0;
	double icUpperBound = 20.0;
};
