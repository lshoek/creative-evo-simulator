#include "OrderlyCoverageEvaluator.h"
#include "Artifact/QuadtreeCompressor/qtree.hpp"
#include "Utils/MathUtils.h"

void OrderlyCoverageEvaluator::setup(uint32_t width, uint32_t height)
{
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

	double imageComplexity = coverageFunc(coverage);

	// Lower contrast to improve the reliability of the fractal compression algorithm (alpha[1.0-3.0], beta[0-100])
	cv::Mat src;
	cv::Mat blur;

	cv::blur(im, blur, cv::Size(9, 9));
	blur.convertTo(src, CV_8UC1);

	// Quadtree Complexity
	int* pixels = new int[src.rows * src.cols];
	for (int r = 0; r < src.rows; r++) {
		for (int c = 0; c < src.cols; c++) {
			pixels[r * src.cols + c] = src.at<uint8_t>(r, c);
		}
	}
	qt_node* q = init_node();
	add_values(q, pixels, src.rows, src.cols);

	// 0 = very little compression, 20 = a lot of compression
	build_tree(q, 20);

	ofLog() << "Nodes: " << count_tree_nodes(q) << endl;
	ofLog() << "Pixel check: " << check_num_pixels(q) << endl;

	cv::Mat im_unpacked = cv::Mat(src.rows, src.cols, CV_8UC1);

	unpack_tree(q, im_unpacked, src.rows, src.cols);
	cv::imwrite("data/keep/compressed.bmp", im_unpacked);
	delete[] pixels;

	double numNodes = count_tree_nodes(q);
	double numPixels = check_num_pixels(q);
	double processingComplexity = (numPixels - numNodes) / numPixels;

	double fitness = imageComplexity / processingComplexity;

	ofLog() << "numPixels: " << numPixels << " , numNodes: " << numNodes;
	ofLog() << "ic: " << imageComplexity << " , pc: " << processingComplexity << " , f: " << fitness;

	return fitness;
}

double OrderlyCoverageEvaluator::coverageFunc(double coverage)
{
	double x = std::min(coverage, 0.25);
	double y = 1.0 - std::pow(sin(PI*(4.0 * x + 1.0)/2.0), 4.0);

	return std::min(std::max(y, 0.0), 1.0);
}

double OrderlyCoverageEvaluator::getLatestCoverageScore()
{
	return _latestCoverageScore;
}

void OrderlyCoverageEvaluator::setDecodingDepth(int depth)
{
	_decodingDepth = depth;
}
