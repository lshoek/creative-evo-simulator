#include "AestheticEvaluator.h"
#include "Utils/MathUtils.h"

void AestheticEvaluator::setup(uint32_t width, uint32_t height)
{
	_compressor.setup(_pcBlockSize);
	_compressor.setWriteImageToDisk(_bWriteToDisk);
	_compressor.setWriteEncodingToDisk(_bWriteEncodingToDisk);
}

std::vector<double> AestheticEvaluator::evaluate(cv::Mat im)
{
	if (im.elemSize() != 1) {
		ofLog() << "[Evaluator] Warning: elemSize of im is not equal to 1.";
	}
	cv::Mat procImg;
	cv::resize(im, procImg, _processingSize);

	double maxCoverage = procImg.total() * 255.0;
	size_t rawSize = procImg.total();

	// Coverage measure
	double coverage = cv::mean(procImg)[0] / 255.0;
	double coverageReward = coverageFunc(coverage);

	// Image Complexity

	cv::Mat edges, edges_abs;
	cv::Laplacian(procImg, edges, CV_64FC1, 3);
	cv::convertScaleAbs(edges, edges_abs);
	
	// The edgeRate of noise would be ~0.5, therefore IC = edgeRate*2.0
	double edgeRate = cv::sum(edges_abs)[0]/(double)maxCoverage;
	double IC = std::min(edgeRate, 0.5) * 2.0;

	// Processing Complexity -- Fractal method

	// Lower contrast to improve the reliability of the fractal compression algorithm (alpha[1.0-3.0], beta[0-100])
	cv::Mat pcImage, pcBlur;
	cv::blur(im, pcBlur, cv::Size(3, 3));
	pcBlur.convertTo(pcImage, CV_8UC1, 0.9, 20.0);

	_compressor.allocate(pcImage);
	size_t encodingSize = _compressor.getEncodingBytes();

	_compressor.encode();
	_compressor.decode(_decodingDepth - _decodingLevelDiff);
	cv::Mat fractalMat_t0 = _compressor.getDecodedImage();

	_compressor.decode(_decodingLevelDiff);
	cv::Mat fractalMat_t1 = _compressor.getDecodedImage();

	cv::Mat diff;
	cv::Mat diffConverted;
	cv::Mat se(pcImage.rows, pcImage.cols, CV_64FC1);

	// PCt0
	cv::absdiff(pcImage, fractalMat_t0, diff);
	diff.convertTo(diffConverted, CV_64FC1);
	cv::pow(diffConverted, 2, se);

	double rmsePCt0 = sqrt(cv::mean(se)[0]);
	double PCt0 = glm::max(rmsePCt0, _pcLowerBound);

	// PCt1
	cv::absdiff(pcImage, fractalMat_t1, diff);
	diff.convertTo(diffConverted, CV_64FC1);
	cv::pow(diffConverted, 2, se);

	double rmsePCt1 = sqrt(cv::mean(se)[0]);
	double PCt1 = glm::max(rmsePCt1, _pcLowerBound);

	// Total Fitness
	bool bDiscard = false;
	double eps = 0.0005;
	double aestheticReward = 0.0;

	double PCdiffRaw = PCt1 - PCt0;
	if (PCt0 > PCt1) {
		ofLog() << "DISCARDED: PCt0 > PCt1";
		bDiscard = true;
	}

	if (!bDiscard) {
		double PCdiff = std::min(PCt1 - PCt0, eps);

		double term_a = std::pow(IC, _a);
		double term_b = std::pow(PCt0 * PCt1, _b);
		double term_c = std::pow(PCdiff / PCt1, _c);

		double result = term_a / (term_b * term_c);
		if (!isnan(result)) {
			aestheticReward = result;
		}
	}
	double weightedAestheticReward = aestheticReward * coverageReward;

	// Make aesthetic reward partly proportional to the coverage reward to prevent high rewards for low coverage artifacts
	double fitness = weightedAestheticReward + coverageReward * _fitnessMult * !bDiscard;

	char msg[512];
	sprintf(msg,
		"\nEvaluation Report:\ncoverage: %.4f%% -> %.4f\nIC (laplace): %.4f; PCt0: %.4f; PCt1: %.4f; diff: %.4f\naestheticReward:%.4f -> %.4f\nfitness: %.4f",
		coverage*100.0, coverageReward,
		IC, PCt0, PCt1, PCdiffRaw,
		aestheticReward, weightedAestheticReward, 
		fitness
	);
	ofLog() << msg << std::endl;

	if (_bWriteToDisk) {
		cv::imwrite("data/keep/eval_out_lvl1.bmp", fractalMat_t0);
		cv::imwrite("data/keep/eval_out_lvl2.bmp", fractalMat_t1);
		cv::imwrite("data/keep/eval_out_laplace.bmp", edges);
	}

	std::vector<double> result(6);
	if (bDiscard) {
		result[0] = 0.0;
		return result;
	}
	result[0] = fitness;
	result[1] = coverage;
	result[2] = coverageReward;
	result[3] = IC;
	result[4] = PCt0;
	result[5] = PCt1;

	return result;
}

double AestheticEvaluator::coverageFunc(double coverage)
{
	double p = _peakCoverage; // peak reward for coverage
	double c = std::min(coverage, p);
	double y = 1.0 - std::pow(sin(PI * ((1.0/p) * c + 1.0) / 2.0), 4.0);

	return std::min(std::max(y, 0.0), 1.0);
}

void AestheticEvaluator::setDecodingDepth(int depth)
{
	_decodingDepth = depth;
}
