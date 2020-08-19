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
	cv::Mat diff;
	cv::Mat diffConverted;
	cv::Mat se(im.rows, im.cols, CV_64FC1);
	cv::Mat se_jpeg(im.rows, im.cols, CV_64FC1);

	// Coverage measure
	double coverage = cv::mean(im)[0] / 255.0;
	double coverageReward = coverageFunc(coverage);


	// Image Complexity -- JPEG Sobel method

	cv::Mat grad, grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;

	cv::Sobel(im, grad_x, CV_16S, 1, 0, 1);
	cv::Sobel(im, grad_y, CV_16S, 0, 1, 1);
	cv::convertScaleAbs(grad_x, abs_grad_x);
	cv::convertScaleAbs(grad_y, abs_grad_y);
	cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

	ofPixels sobelPixBuffer;
	sobelPixBuffer.setFromExternalPixels(grad.ptr(), grad.rows, grad.cols, ofPixelFormat::OF_PIXELS_GRAY);

	ofImageLoadSettings settings;
	settings.grayscale = true;

	ofPixels jpegPixBuffer;
	ofBuffer jpegBuffer;
	ofSaveImage(sobelPixBuffer, jpegBuffer, ofImageFormat::OF_IMAGE_FORMAT_JPEG, ofImageQualityType::OF_IMAGE_QUALITY_HIGH);
	ofLoadImage(jpegPixBuffer, jpegBuffer, settings);

	cv::Mat jpegMat(grad.rows, grad.cols, CV_8UC1, jpegPixBuffer.getData());

	cv::absdiff(grad, jpegMat, diff);
	diff.convertTo(diffConverted, CV_64F);

	cv::pow(diffConverted, 2, se_jpeg);
	double rmseIC = sqrt(cv::mean(se_jpeg)[0]);
	double compressionRatioIC = (double)im.total() / jpegBuffer.size();
	double IC = rmseIC / compressionRatioIC;


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

	// PCt0
	cv::absdiff(pcImage, fractalMat_t0, diff);
	diff.convertTo(diffConverted, CV_64FC1);
	cv::pow(diffConverted, 2, se);

	double rmsePCt0 = sqrt(cv::mean(se)[0]);
	double PCt0 = glm::max(1.0/rmsePCt0, _pcLowerBound);

	// PCt1
	cv::absdiff(pcImage, fractalMat_t1, diff);
	diff.convertTo(diffConverted, CV_64FC1);
	cv::pow(diffConverted, 2, se);

	double rmsePCt1 = sqrt(cv::mean(se)[0]);
	double PCt1 = glm::max(1.0/rmsePCt1, _pcLowerBound);

	// Total Fitness
	bool bDiscard = false;
	double eps = 0.0005;
	double aestheticReward = 0.0;

	double PCdiffRaw = PCt1 - PCt0;
	if (PCt0 > PCt1) {
		ofLog() << "DISCARDED: 1.0/rmsePCt0 > 1.0/rmsePCt1";
		bDiscard = true;
	}

	double PC = 0.0;
	if (!bDiscard) {
		double PCdiff = std::max(PCt1 - PCt0, eps);

		double term_a = std::pow(IC, _a);
		double term_b = std::pow(PCt0 * PCt1, _b);
		double term_c = std::pow(PCdiff / PCt1, _c);
		PC = term_b * term_c;

		double result = term_a / PC;
		if (!isnan(result)) {
			aestheticReward = result;
		}
	}

	// Make aesthetic reward partly proportional to the coverage reward to prevent high rewards for low coverage artifacts
	double fitness = coverageReward * 100.0 + (aestheticReward * coverageReward) * !bDiscard;

	char msg[512];
	sprintf(msg,
		"\nEvaluation Report:\ncoverage: %.4f%% -> %.4f\nIC (Sobel/JPEG): %.4f; PC: %.4f; PCdiff: %.4f\naestheticReward:%.4f\nfitness: %.4f",
		coverage*100.0, coverageReward,
		IC, PC, PCdiffRaw,
		aestheticReward, 
		fitness
	);
	ofLog() << msg << std::endl;

	if (_bWriteToDisk) {
		cv::imwrite("data/keep/eval_out_jpeg.bmp", jpegMat);
		cv::imwrite("data/keep/eval_out_se_jpeg.bmp", se_jpeg);
		cv::imwrite("data/keep/eval_out_lvl1.bmp", fractalMat_t0);
		cv::imwrite("data/keep/eval_out_lvl2.bmp", fractalMat_t1);
		cv::imwrite("data/keep/eval_out_sobel_xy.bmp", grad);
	}

	std::vector<double> result(7);
	if (bDiscard) {
		result[0] = 0.0;
		return result;
	}
	result[0] = fitness;
	result[1] = coverage;
	result[2] = coverageReward;
	result[3] = IC;
	result[4] = PC;
	result[5] = PCt0;
	result[6] = PCt1;

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
