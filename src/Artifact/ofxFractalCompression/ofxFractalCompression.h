/// 
/// Addon: lshoek
///	Original code author: wodonggun 
///	https://github.com/wodonggun/Fractal-Compression-openCV
///

#pragma once
#include "EncodingResult.h"
#include "ofPixels.h"
#include "ofxOpenCv.h"
#include "ofFileUtils.h"

class ofxFractalCompression
{
public:
	void setup(int blockSize);
	bool allocate(cv::Mat im);

	void encode();
	void decode(int depth);
	void decodeFromFile(int depth);

	const EncodingResult* getEncodingResultPtr() const;
	int getNumBlocks();

	size_t getEncodingBytes();
	cv::Mat getDecodedImage();

	void setLog(bool enable);
	void setWriteEncodingToDisk(bool enable);
	void setWriteImageToDisk(bool enable);

	void dealloc();

private:
	EncodingResult** m_imageEncoding;
	int** m_imageData;
	int** m_decodedImageData;

	int m_imageWidth = 0;
	int m_imageHeight = 0;

	int m_blockSize = 8;
	int m_numBlocks = 0;
	int m_encodingBytes = 0;
	int m_currentDecodingDepth = 0;

	bool m_bLog = false;
	bool m_bEncToDisk = false;
	bool m_bImageToDisk = false;

	bool m_bMemoryAllocated = false;

	EncodingResult TemplateMatchingWithDownSamplingPlusShuffle_StructEncoding(int** block, int bx, int by, int** image, int width, int height, double alpha);
	void Decoding(EncodingResult** en_Result, int** image_dec, int width, int height, int size_x, int size_y);

	void printEncodingResult(EncodingResult** en_result, int block_x, int block_y);
};
