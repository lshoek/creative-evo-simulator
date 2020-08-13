#include "AestheticEvaluator.h"
#include "Utils/MathUtils.h"

void AestheticEvaluator::setup(uint32_t width, uint32_t height)
{
	_compressor.setup(8);
	_compressor.setWriteImageToDisk(false);
	_compressor.setWriteEncodingToDisk(false);

	_maxCoverage = width * height * 255.0;
}

double AestheticEvaluator::evaluate(cv::Mat im)
{
	if (im.elemSize() != 1) {
		ofLog() << "[Evaluator] Warning: elemSize of im is not equal to 1.";
	}
	if (im.total() * 255.0 != _maxCoverage) {
		_maxCoverage = im.total() * 255.0;
	}
	size_t rawSize = im.total();

	// Coverage measure
	double coverage = cv::mean(im)[0] / 255.0;
	double coverageReward = coverageFunc(coverage);

	// Image Complexity

	cv::Mat edges, edges_abs;
	cv::Laplacian(im, edges, CV_64FC1, 3);
	cv::convertScaleAbs(edges, edges_abs);
	
	// The edgeRate of noise would be ~0.5, therefore IC = edgeRate*2.0
	double edgeRate = cv::sum(edges_abs)[0]/(double)_maxCoverage;
	double IC = std::min(edgeRate, 0.5) * 2.0;

	// Processing Complexity -- Fractal method

	// Lower contrast to improve the reliability of the fractal compression algorithm (alpha[1.0-3.0], beta[0-100])
	cv::Mat pcImage, blur;
	cv::blur(im, blur, cv::Size(5, 5));
	blur.convertTo(pcImage, CV_8UC1);

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
	double PCt0 = glm::max(rmsePCt0, pc0LowerBound);

	// PCt1
	cv::absdiff(pcImage, fractalMat_t1, diff);
	diff.convertTo(diffConverted, CV_64FC1);
	cv::pow(diffConverted, 2, se);

	double rmsePCt1 = sqrt(cv::mean(se)[0]);
	double PCt1 = glm::max(rmsePCt1, pc1Lowerbound);

	// Complexity Bias
	double a = 1.0; // a = 2.0 grants a lot of extra fitness for higher IC
	double b = 0.4;
	double c = 0.2;

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

		double term_a = std::pow(IC, a);
		double term_b = std::pow(PCt0 * PCt1, b);
		double term_c = std::pow(PCdiff / PCt1, c);

		double result = term_a / (term_b * term_c);
		if (!isnan(result)) {
			aestheticReward = result;
		}
	}
	double weightedAestheticReward = aestheticReward * coverageReward;

	// Make aesthetic reward partly proportional to the coverage reward to prevent high rewards for low coverage artifacts
	double fitness = weightedAestheticReward + coverageReward * 1000.0 * !bDiscard;

	char msg[512];
	sprintf(msg,
		"\nEvaluation Report:\ncoverage: %.4f%% -> %.4f\nIC (laplace): %.4f; PCt0: %.4f; PCt1: %.4f; diff: %.4f\naestheticReward:%.4f -> %.4f\nfitness: %.4f",
		coverage*100.0, coverageReward,
		IC, PCt0, PCt1, PCdiffRaw,
		aestheticReward, weightedAestheticReward, 
		fitness
	);
	ofLog() << msg << std::endl;

	//cv::imwrite("data/keep/eval_out_lvl1.bmp", fractalMat_t0);
	//cv::imwrite("data/keep/eval_out_lvl2.bmp", fractalMat_t1);
	//cv::imwrite("data/keep/eval_out_laplace.bmp", edges);

	if (bDiscard) {
		return 0.0;
	}
	return fitness;
}

double AestheticEvaluator::coverageFunc(double coverage)
{
	double x = std::min(coverage, 0.25);
	double y = 1.0 - std::pow(sin(PI * (4.0 * x + 1.0) / 2.0), 4.0);

	return std::min(std::max(y, 0.0), 1.0);
}

void AestheticEvaluator::setDecodingDepth(int depth)
{
	_decodingDepth = depth;
}
