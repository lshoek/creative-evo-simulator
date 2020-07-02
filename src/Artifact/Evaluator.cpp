#include "Evaluator.h"

void Evaluator::setup()
{
	_compressor.setup(8);
	_compressor.setWriteImageToDisk(true);
	_compressor.setWriteEncodingToDisk(true);
}

double Evaluator::evaluate(cv::Mat im)
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

	// Processing Complexity -- Fractal method 
	_compressor.allocate(srcImage);
	_compressor.encode();
	_compressor.decode(6);

	cv::Mat fractalMat = _compressor.getDecodedImage();
	cv::subtract(srcImage, fractalMat, diff);
	cv::pow(diff, 2, se);

	size_t encodingSize = _compressor.getEncodingBytes();
	double rmsePC = sqrt(cv::sum(se)[0] / rawSize);
	double compRatioPC = encodingSize / (double)rawSize;

	double PC = rmsePC / compRatioPC;

	double fitness = IC/PC;

	char msg[256];
	sprintf(msg, "Artifact Evaluation Report:\nrmse[jpeg]: %f\ncompratio[jpeg]: %f\nIC: %f\nrmse[fract]: %f\ncompratio[fract]: %f\nPC: %f\nfitness: %f\n", 
		rmseIC, compRatioIC, IC, rmsePC, compRatioPC, PC, fitness
	);
	ofLog() << msg;

	return fitness;
}
