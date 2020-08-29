#pragma once
#include "Artifact/EvaluatorBase.h"
#include "Artifact/FractalCompressor/FractalCompressor.h"
#include "ofxOpenCv.h"

class AestheticEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override;
	virtual std::vector<double> evaluate(cv::Mat im) override;

	void setDecodingDepth(int depth);

private:

	// Remaps coverage to a biased curve [0..1]
	double coverageFunc(double coverage);

	FractalCompressor _compressor;

	cv::Size _processingSize = cv::Size(256, 256);

	int _pcBlockSize = 8;
	int _decodingDepth = 4;
	int _decodingLevelDiff = 2;

	double _peakCoverage = 0.06125;

	// Aesthetic Fitness Terms (Normal: _a=1.0; High Complexity: _a=1.5)
	double _a = 1.0;	// (a > 1.0 grants a lot of extra fitness for higher IC)
	double _b = 0.6;	// 0.4 
	double _c = 0.3;	// 0.2

	double _pcLowerBound = 0.0001;

	bool _bWriteToDisk = true;
	bool _bWriteEncodingToDisk = false;
};
