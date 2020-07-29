#include "AestheticEvaluator.h"

void AestheticEvaluator::setup(uint32_t width, uint32_t height)
{
	_compressor.setup(8);
	_compressor.setWriteImageToDisk(true);
	_compressor.setWriteEncodingToDisk(true);
}

double AestheticEvaluator::evaluate(cv::Mat im)
{
	if (im.elemSize() != 1) {
		ofLog() << "[Evaluator] Warning: elemSize of im is not equal to 1.";
	}
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
	cv::imwrite("data/jpeg_out.bmp", jpegMat);
	
	cv::Mat diff;
	cv::Mat se;

	cv::subtract(srcImage, jpegMat, diff);
	cv::pow(diff, 2, se);

	double rmseIC = sqrt(cv::sum(se)[0]/rawSize);
	double compRatioIC = jpegBuffer.size()/(double)rawSize;

	double IC = rmseIC / compRatioIC;

	// Filter complexities outside of acceptable bounds
	if (IC < minComplexity || IC > maxComplexity) {
		ofLog() << "Artifact discarded -> IC: " << IC;
		return 0.0;
	}

	// Processing Complexity -- Fractal method 

	_compressor.allocate(srcImage);
	size_t encodingSize = _compressor.getEncodingBytes();

	_compressor.encode();

	_compressor.decode(_decodingDepth);
	cv::Mat fractalMat_t0 = _compressor.getDecodedImage();

	_compressor.decode(1);
	cv::Mat fractalMat_t1 = _compressor.getDecodedImage();

	// PCt0
	cv::subtract(srcImage, fractalMat_t0, diff);
	cv::pow(diff, 2, se);

	double rmsePCt0 = sqrt(cv::sum(se)[0] / rawSize);
	double compRatioPCt0 = encodingSize / (double)rawSize;

	double PCt0 = glm::max(rmsePCt0 / compRatioPCt0, pc0LowerBound);

	// PCt1
	cv::subtract(srcImage, fractalMat_t1, diff);
	cv::pow(diff, 2, se);

	double rmsePCt1 = sqrt(cv::sum(se)[0] / rawSize);
	double compRatioPCt1 = encodingSize / (double)rawSize;

	double PCt1 = glm::max(rmsePCt1 / compRatioPCt1, pc1Lowerbound);

	// Complexity Bias
	double a = 1.0;
	double b = 0.4;
	double c = 0.2;

	// Total Fitness
	double fitness = std::pow(IC, a) / std::pow(PCt0 * PCt1, b) * std::pow((PCt1 - PCt0) / PCt1, c);

	char msg[256];
	sprintf(msg, "Artifact Evaluation Report:\nrmse[jpeg]: %f\ncompratio[jpeg]: %f\nIC: %f\nrmse[fract/t0]: %f\ncompratio[fract/t0]: %f\nPC/t0: %f\nrmse[fract/t1]: %f\ncompratio[fract/t1]: %f\nPC/t1: %f\nfitness: %f\n", 
		rmseIC, compRatioIC, IC, rmsePCt0, compRatioPCt0, PCt0, rmsePCt1, compRatioPCt1, PCt1, fitness
	);
	ofLog() << msg;

	return fitness;
}

void AestheticEvaluator::setDecodingDepth(int depth)
{
	_decodingDepth = depth;
}
