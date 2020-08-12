/// 
/// Addon: lshoek
///	Original code author: wodonggun 
///	https://github.com/wodonggun/Fractal-Compression-openCV
///

#pragma once
#include <stdio.h>

#include "FractalCompressor.h"
#include "FractalCompressorCommon.h"

void FractalCompressor::setup(int blockSize)
{
	m_blockSize = blockSize;
}

bool FractalCompressor::allocate(cv::Mat im)
{
	if (m_bMemoryAllocated) {
		dealloc();
		m_bMemoryAllocated = false;
	}
	if (m_blockSize > 0) {
		m_imageWidth = im.rows;
		m_imageHeight = im.cols;

		m_imageData = ConvertFromMat(im);
		m_decodedImageData = IntAlloc2(m_imageWidth, m_imageHeight);
		for (int i = 0; i < m_imageHeight; i++) {
			for (int j = 0; j < m_imageWidth; j++) {
				m_decodedImageData[i][j] = 0x80;
			}
		}
		m_imageEncoding = ERAlloc2(m_imageWidth / m_blockSize, m_imageHeight / m_blockSize);
		m_numBlocks = (m_imageWidth / m_blockSize) * (m_imageHeight / m_blockSize);
		m_encodingBytes = m_numBlocks * sizeof(EncodingResult);

		m_currentDecodingDepth = 0;
		m_bMemoryAllocated = true;
		return true;
	}
	else return false;
}

void FractalCompressor::encode()
{
	int** block_temp = (int**)IntAlloc2(m_blockSize, m_blockSize);

	for (int i = 0; i < m_imageHeight / m_blockSize; i++) {
		for (int j = 0; j < m_imageWidth / m_blockSize; j++) {
			ReadBlock(m_imageData, m_blockSize * j, m_blockSize * i, m_blockSize, m_blockSize, block_temp);
			m_imageEncoding[i][j] = TemplateMatchingWithDownSamplingPlusShuffle_StructEncoding(block_temp, m_blockSize, m_blockSize, m_imageData, m_imageWidth, m_imageHeight, 1);

			if (m_bLog) {
				printEncodingResult(m_imageEncoding, i, j);
			}
		}
	}
	if (m_bEncToDisk) {
		WriteParameter("data/keep/encoding.txt", m_imageEncoding, m_imageWidth / m_blockSize, m_imageHeight / m_blockSize);
	}
	IntFree2(block_temp, m_blockSize, m_blockSize);
}

void FractalCompressor::decode(int depth)
{
	for (int i = 0; i < depth; i++) {
		Decoding(m_imageEncoding, m_decodedImageData, m_imageWidth, m_imageHeight, m_blockSize, m_blockSize);
		m_currentDecodingDepth++;
		if (m_bImageToDisk) {
			std::string fname = "data/keep/decoded_out_" + ofToString(m_currentDecodingDepth) + ".bmp";
			cv::imwrite(fname, getDecodedImage());
		}
	}
}

void FractalCompressor::decodeFromFile(int depth)
{
	EncodingResult** en_result = ERAlloc2(m_imageWidth / m_blockSize, m_imageHeight / m_blockSize);
	ReadParameter("data/keep/encoding.txt", en_result, m_imageWidth / m_blockSize, m_imageHeight / m_blockSize);

	for (int i = 0; i < depth; i++) {
		Decoding(en_result, m_decodedImageData, m_imageWidth, m_imageHeight, m_blockSize, m_blockSize);
		if (m_bImageToDisk) {
			std::string fname = "data/keep/decoded_out_" + ofToString(i) + ".bmp";
			cv::imwrite(fname, getDecodedImage());
		}
	}
}

EncodingResult FractalCompressor::TemplateMatchingWithDownSamplingPlusShuffle_StructEncoding(int** block, int bx, int by, int** image, int width, int height, double alpha)
{
	EncodingResult struct_Tmp;
	int error_min = INT_MAX;

	int** temp = (int**)IntAlloc2(bx * 2, by * 2);
	int** domain = (int**)IntAlloc2(bx, by);
	int** tmp_test = (int**)IntAlloc2(bx, by);
	int block_avg = ComputeAVG(block, bx, by);
	struct_Tmp.avg = block_avg;
	int** block_AC = (int**)IntAlloc2(bx, by);
	int** domain_AC = (int**)IntAlloc2(bx, by);

	Copy_img(block, bx, by, block_AC);
	Find_AC(block_AC, bx, by, block_avg);

	for (int i = 0; i < height - (by * 2); i += by) {
		for (int j = 0; j < width - (bx * 2); j += bx) {
			ReadBlock(image, j, i, bx * 2, by * 2, temp);
			Contraction(temp, domain, bx * 2, by * 2);
			int domain_avg = ComputeAVG(domain, bx, by);
			for (int n = 0; n < 8; n++) {
				Isometry(n, domain, bx, by, tmp_test);
				Find_AC(tmp_test, bx, by, domain_avg);
				for (double d = 0.3; d <= 1.0; d += 0.1) {
					AC_control(tmp_test, bx, by, d, domain_AC);
					int error = ComputeAbsError(block_AC, bx, domain_AC, bx, by, 0, 0);
					if (error < error_min) {
						error_min = error;
						struct_Tmp.x = j;
						struct_Tmp.y = i;
						struct_Tmp.alpha = d;
						struct_Tmp.geo = n;
						struct_Tmp.error = error;
					}
				}
			}
		}
	}
	IntFree2(temp, bx * 2, by * 2);
	IntFree2(domain, bx, by);
	IntFree2(block_AC, bx, by);
	IntFree2(domain_AC, bx, by);
	IntFree2(tmp_test, bx, by);

	return struct_Tmp;
}

void FractalCompressor::Decoding(EncodingResult** en_Result, int** image_dec, int width, int height, int size_x, int size_y)
{
	int** block = IntAlloc2(size_x * 2, size_y * 2);
	int** block_contract_tmp = IntAlloc2(size_x, size_y);
	int** block_contract_tmp_aftercontrol = IntAlloc2(size_x, size_y);
	int** image_dec_tmp = IntAlloc2(width, height);

	for (int i = 0; i < height / size_y; i++) {
		for (int j = 0; j < width / size_x; j++) {
			ReadBlock(image_dec, en_Result[i][j].x, en_Result[i][j].y, size_x * 2, size_y * 2, block);
			Contraction(block, block_contract_tmp, size_x * 2, size_y * 2);
			int b_avg = ComputeAVG(block_contract_tmp, size_x, size_y);
			Find_AC(block_contract_tmp, size_x, size_y, b_avg);
			Isometry(en_Result[i][j].geo, block_contract_tmp, size_x, size_y, block_contract_tmp_aftercontrol);
			AC_control(block_contract_tmp_aftercontrol, size_x, size_y, en_Result[i][j].alpha, block_contract_tmp_aftercontrol);
			Find_AC(block_contract_tmp_aftercontrol, size_x, size_y, -en_Result[i][j].avg);
			WriteBlock(image_dec_tmp, j * size_x, i * size_y, size_x, size_y, block_contract_tmp_aftercontrol);
		}
	}
	Copy_img(image_dec_tmp, width, height, image_dec);

	IntFree2(block, size_x * 2, size_y * 2);
	IntFree2(block_contract_tmp, size_x, size_y);
	IntFree2(block_contract_tmp_aftercontrol, size_x, size_y);
	IntFree2(image_dec_tmp, width, height);
}

void FractalCompressor::printEncodingResult(EncodingResult** en_result, int block_x, int block_y)
{
	int i = block_x;
	int j = block_y;

	printf(" < %3d , %3d > xǥ : %3d   yǥ : %3d  err : %3d Isom : %3d  alpha : %.1lf  avg : %3d \n",
		j * m_blockSize, i * m_blockSize,
		en_result[i][j].x,
		en_result[i][j].y,
		en_result[i][j].error,
		en_result[i][j].geo,
		en_result[i][j].alpha,
		en_result[i][j].avg
	);
}

const EncodingResult* FractalCompressor::getEncodingResultPtr() const
{
	return &m_imageEncoding[0][0];
}

int FractalCompressor::getNumBlocks()
{
	return m_numBlocks;
}

size_t FractalCompressor::getEncodingBytes()
{
	return m_encodingBytes;
}

cv::Mat FractalCompressor::getDecodedImage()
{
	return ConvertToMat(m_decodedImageData, m_imageWidth, m_imageHeight);
}

void FractalCompressor::setLog(bool enable) {
	m_bLog = enable;
}
void FractalCompressor::setWriteEncodingToDisk(bool enable) {
	m_bEncToDisk = enable;
}
void FractalCompressor::setWriteImageToDisk(bool enable) {
	m_bImageToDisk = enable;
}

void FractalCompressor::dealloc()
{
	if (m_bMemoryAllocated) {
		ERFree2(m_imageEncoding, m_imageWidth / m_numBlocks, m_imageHeight / m_numBlocks);
		IntFree2(m_imageData, m_imageWidth, m_imageHeight);
		IntFree2(m_decodedImageData, m_imageWidth, m_imageHeight);
	}
}
