#include "AestheticEvaluator.h"
#include "Utils/MathUtils.h"

void AestheticEvaluator::setup(uint32_t width, uint32_t height)
{
	_compressor.setup(16);
	_compressor.setWriteImageToDisk(false);
	_compressor.setWriteEncodingToDisk(false);

	_maxCoverageReward = width * height * 255.0;
}

double AestheticEvaluator::evaluate(cv::Mat im)
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

	// Lower contrast to improve the reliability of the fractal compression algorithm (alpha[1.0-3.0], beta[0-100])
	cv::Mat srcImage;
	im.convertTo(srcImage, CV_8UC1, 0.7, 50.0);

	size_t rawSize = srcImage.total();
	ofPixels rawPixBuffer;
	rawPixBuffer.setFromExternalPixels(srcImage.ptr(), srcImage.rows, srcImage.cols, ofPixelFormat::OF_PIXELS_GRAY);

	// Image Complexity -- JPEG method 
	ofImageLoadSettings settings;
	settings.grayscale = true;

	ofPixels jpegPixBuffer;
	ofBuffer jpegBuffer;
	ofSaveImage(rawPixBuffer, jpegBuffer, ofImageFormat::OF_IMAGE_FORMAT_JPEG, ofImageQualityType::OF_IMAGE_QUALITY_HIGH);
	ofLoadImage(jpegPixBuffer, jpegBuffer, settings);

	cv::Mat jpegMat(srcImage.rows, srcImage.cols, CV_8UC1, jpegPixBuffer.getData());
	//cv::imwrite("data/keep/jpeg_out.bmp", jpegMat);

	cv::Mat diff;
	cv::Mat diffConverted;
	cv::Mat se(srcImage.rows, srcImage.cols, CV_64F);

	cv::absdiff(srcImage, jpegMat, diff);
	diff.convertTo(diffConverted, CV_64F);

	cv::pow(diffConverted, 2, se);
	//cv::imwrite("data/se.bmp", se);

	double rmseIC = sqrt(cv::mean(se)[0]);
	double compressionRatioIC = (double)rawSize / jpegBuffer.size();
	double IC = rmseIC / compressionRatioIC;

	// Filter complexities outside of acceptable bounds
	if (IC < minComplexity || IC > maxComplexity) {
		ofLog() << "Artifact discarded -> IC: " << IC;
		ofLog() << rawSize << '/' << jpegBuffer.size() << '=' << compressionRatioIC;
		ofLog() << rmseIC << '/' << compressionRatioIC << '=' << IC;
		return 0.0;
	}

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

	// Complexity Bias
	double a = 1.0;
	double b = 0.4;
	double c = 0.2;

	// Coverage Reward
	double coverageReward = coverageFunc(coverage);

	// Total Fitness
	double eps = 0.0005;
	double aestheticReward = 0.0;

	if (PCt0 > PCt1) {
		ofLog() << "Warning: PCt0 > PCt1!";
	}

	if (abs(PCt0 - PCt1) > eps) {
		double term_a = std::pow(IC, a);
		double term_b = std::pow(PCt0 * PCt1, b);
		double term_c = std::pow(abs(PCt1 - PCt0) / PCt1, c); // abs(x) because PCt0 can be higher than PCt1 causing pow(x, double) to fail

		double result = term_a / (term_b * term_c);
		if (!isnan(result)) {
			aestheticReward = result;
		}
	}
	double weightedAestheticReward = aestheticReward * coverageReward;

	// Make aesthetic reward partly proportional to the coverage reward to prevent high rewards for low coverage artifacts
	double fitness = weightedAestheticReward + coverageReward;

	char msg[512];
	sprintf(msg,
		"Artifact Evaluation Report:\ncoverage:%.4f\ncoverageReward:%.4f\nrmse[jpeg]: %.4f\ncompressionRatio[jpeg]: %.4f\nIC: %.4f\nrmse[fract/t0]: %.4f\nPC/t0: %.4f\nrmse[fract/t1]: %.4f\nPC/t1: %.4f\naestheticReward:%.4f\nweightedAestheticReward:%.4f\nfitness: %.4f",
		coverage, coverageReward,
		rmseIC, compressionRatioIC, IC,
		rmsePCt0, PCt0,
		rmsePCt1, PCt1,
		aestheticReward, weightedAestheticReward, fitness
	);
	ofLog() << msg << std::endl;

	return fitness;
}

double AestheticEvaluator::coverageFunc(double coverage)
{
	double x = coverage;
	double k = 0.0;			// center
	double q = 3.0;			// stretches the curve
	if (x > 0) {
		return std::min(7.7478 * MathUtils::normPDF(q * x, 0.0, 1.0) * MathUtils::absCurve(q * x, k), 1.0);
	}
	return 0.0;
}

double AestheticEvaluator::getLatestCoverageScore()
{
	return _latestCoverageScore;
}

void AestheticEvaluator::setDecodingDepth(int depth)
{
	_decodingDepth = depth;
}
