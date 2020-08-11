#include "OrderlyCoverageEvaluator.h"
#include "Utils/MathUtils.h"

void OrderlyCoverageEvaluator::setup(uint32_t width, uint32_t height)
{
	_compressor.setup(16);
	_compressor.setWriteImageToDisk(false);
	_compressor.setWriteEncodingToDisk(false);

	_maxCoverageReward = width * height * 255.0;
}

double OrderlyCoverageEvaluator::evaluate(cv::Mat im)
{
	if (im.elemSize() != 1) {
		ofLog() << "[Evaluator] Warning: elemSize of im is not equal to 1.";
	}
	if (im.rows * im.cols * 255.0 != _maxCoverageReward) {
		_maxCoverageReward = im.rows * im.cols * 255.0;
	}

	// Coverage measure
	double coverage = cv::mean(im)[0] / 255.0;
	_latestCoverageScore = coverage;

	double coverageReward = coverageFunc(coverage);

	// Lower contrast to improve the reliability of the fractal compression algorithm (alpha[1.0-3.0], beta[0-100])
	cv::Mat srcImage;
	im.convertTo(srcImage, CV_8UC1, 0.7, 50.0);

	cv::Mat diff;
	cv::Mat diffConverted;
	cv::Mat se(srcImage.rows, srcImage.cols, CV_64F);

	// Processing Complexity -- Fractal method
	_compressor.allocate(srcImage);
	size_t encodingSize = _compressor.getEncodingBytes();

	// We do not have to calculate the compression ratio as it will be same for both images, whatever it is.
	double compressionRatio = 1.0;

	_compressor.encode();
	_compressor.decode(_decodingDepth - 1);
	cv::Mat fractalMat_t0 = _compressor.getDecodedImage();
	cv::imwrite("data/keep/1.bmp", fractalMat_t0);

	_compressor.decode(2); // 1
	cv::Mat fractalMat_t1 = _compressor.getDecodedImage();
	cv::imwrite("data/keep/2.bmp", fractalMat_t1);

	// PCt0
	cv::absdiff(srcImage, fractalMat_t0, diff);
	diff.convertTo(diffConverted, CV_64F);
	cv::pow(diffConverted, 2, se);

	double rmsePCt0 = sqrt(cv::mean(se)[0]);
	double PCt0 = glm::max(rmsePCt0 / compressionRatio, pc0LowerBound);

	// PCt1
	cv::absdiff(srcImage, fractalMat_t1, diff);
	diff.convertTo(diffConverted, CV_64F);
	cv::pow(diffConverted, 2, se);

	double rmsePCt1 = sqrt(cv::mean(se)[0]);
	double PCt1 = glm::max(rmsePCt1 / compressionRatio, pc1Lowerbound);

	// Total Fitness
	double eps = 0.0005;
	double PCdiff = std::max(abs(PCt0 - PCt1), eps);
	double fitness = (coverage / 2.0 * std::pow(0.1, 0.5 * PCdiff)) * 1000.0;

	char msg[512];
	sprintf(msg,
		"Artifact Evaluation Report:\ncoverage:%.4f\nrmse[fract/t0]: %.4f\nPC/t0: %.4f\nrmse[fract/t1]: %.4f\nPC/t1: %.4f\nfitness: %.4f",
		coverage,
		rmsePCt0, PCt0,
		rmsePCt1, PCt1,
		fitness
	);
	ofLog() << msg << std::endl;

	return fitness;
}

double OrderlyCoverageEvaluator::coverageFunc(double coverage)
{
	double x = coverage;
	double k = 0.0;			// center
	double q = 3.0;			// stretches the curve
	if (x > 0) {
		return std::min(7.7478 * MathUtils::normPDF(q * x, 0.0, 1.0) * MathUtils::absCurve(q * x, k), 1.0);
	}
	return 0.0;
}

double OrderlyCoverageEvaluator::getLatestCoverageScore()
{
	return _latestCoverageScore;
}

void OrderlyCoverageEvaluator::setDecodingDepth(int depth)
{
	_decodingDepth = depth;
}
