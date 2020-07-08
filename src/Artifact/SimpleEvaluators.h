#pragma once
#include "Artifact/EvaluatorBase.h"
#include "ofxOpenCv.h"

class CoverageEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override 
	{
		_maxReward = width * height * 255.0;
	};

	virtual double evaluate(cv::Mat im) override 
	{
		double total = cv::sum(im)[0];
		double fitness = total / _maxReward;
		return fitness;
	};
private:
	double _maxReward = 255.0;
};

class CircleCoverageEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override 
	{
		_maskMat = cv::Mat(width, height, CV_8UC1);
		_maskMat = cv::Scalar(0);

		cv::circle(_maskMat, cv::Point(_maskMat.rows / 2, _maskMat.cols / 2), _maskMat.cols / 4, cv::Scalar(255), cv::FILLED);
		cv::bitwise_not(_maskMat, _invMaskMat);

		// Calculate maximum reward/fitness
		for (uint32_t i = 0; i < _maskMat.total(); i++) {
			_maxReward += _maskMat.at<uchar>(i);
		}

		// Debug Canvas Evaluation Mask
		_cvDebugImage.allocate(_maskMat.rows, _maskMat.cols);
		_cvDebugImage.setFromPixels(_maskMat.ptr<uchar>(), _maskMat.rows, _maskMat.cols);
	};

	virtual double evaluate(cv::Mat im) override 
	{
		cv::bitwise_and(im, _maskMat, _rewardMat);
		cv::bitwise_and(im, _invMaskMat, _penaltyMat);

		double total = 0.0;
		for (uint32_t i = 0; i < im.total(); i++) {
			total += _rewardMat.at<uchar>(i);
			total -= _penaltyMat.at<uchar>(i);
		}
		double fitness = total / _maxReward;
		_cvDebugImage.setFromPixels(_rewardMat.ptr<uchar>(), im.cols, im.rows);

		return fitness;
	};

private:
	cv::Mat _maskMat, _invMaskMat;
	cv::Mat _rewardMat, _penaltyMat;
	double _maxReward = 255.0;

	ofxCvGrayscaleImage _cvDebugImage;
};

class InverseCircleCoverageEvaluator : public EvaluatorBase
{
public:
	virtual void setup(uint32_t width, uint32_t height) override 
	{
		_maskMat = cv::Mat(width, height, CV_8UC1);
		_maskMat = cv::Scalar(0);
		cv::circle(_maskMat, cv::Point(_maskMat.rows / 2, _maskMat.cols / 2), _maskMat.cols / 4, cv::Scalar(255), cv::FILLED);
		cv::bitwise_not(_maskMat, _invMaskMat);

		// Calculate maximum reward/fitness
		for (uint32_t i = 0; i < _invMaskMat.total(); i++) {
			_maxReward += _invMaskMat.at<uchar>(i);
		}

		// Debug Canvas Evaluation Mask
		_cvDebugImage.allocate(_maskMat.rows, _maskMat.cols);
		_cvDebugImage.setFromPixels(_maskMat.ptr<uchar>(), _maskMat.rows, _maskMat.cols);
	};

	virtual double evaluate(cv::Mat im) override 
	{
		cv::bitwise_and(im, _invMaskMat, _rewardMat);
		cv::bitwise_and(im, _maskMat, _penaltyMat);

		double total = 0.0;
		for (uint32_t i = 0; i < im.total(); i++) {
			total += _rewardMat.at<uchar>(i);
			total -= _penaltyMat.at<uchar>(i);
		}
		double fitness = total / _maxReward;
		_cvDebugImage.setFromPixels(_rewardMat.ptr<uchar>(), im.cols, im.rows);

		return fitness;
	};

private:
	cv::Mat _maskMat, _invMaskMat;
	cv::Mat _rewardMat, _penaltyMat;
	double _maxReward = 255.0;

	ofxCvGrayscaleImage _cvDebugImage;
};
