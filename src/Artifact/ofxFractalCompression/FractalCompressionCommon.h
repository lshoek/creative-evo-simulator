/// 
/// Addon: lshoek
///	Original code author: wodonggun 
///	https://github.com/wodonggun/Fractal-Compression-openCV
///

#pragma once
#include "EncodingResult.h"
#include <stdio.h>

int** IntAlloc2(int width, int height)
{
	int** tmp;
	tmp = (int**)calloc(height, sizeof(int*));
	for (int i = 0; i < height; i++) {
		tmp[i] = (int*)calloc(width, sizeof(int));
	}
	return(tmp);
}

void IntFree2(int** image, int width, int height)
{
	for (int i = 0; i < height; i++) {
		free(image[i]);
	}
	free(image);
}

EncodingResult** ERAlloc2(int width, int height)
{
	EncodingResult** tmp;
	tmp = (EncodingResult**)calloc(height, sizeof(EncodingResult*));
	for (int i = 0; i < height; i++) {
		tmp[i] = (EncodingResult*)calloc(width, sizeof(EncodingResult));
	}
	return(tmp);
}

void ERFree2(EncodingResult** image, int width, int height)
{
	for (int i = 0; i < height; i++) {
		free(image[i]);
	}
	free(image);
}

int** ConvertFromMat(cv::Mat img)
{
	int** image = (int**)IntAlloc2(img.cols, img.rows);
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			image[i][j] = img.at<unsigned char>(i, j);
		}
	}
	return(image);
}

cv::Mat ConvertToMat(int** image, int width, int height)
{
	cv::Mat img(width, height, CV_8UC1);
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			img.at<unsigned char>(i, j) = (unsigned char)image[i][j];
		}
	}
	return img;
}

int** ReadImage(const char* name, int* width, int* height)
{
	cv::Mat img = cv::imread(name, cv::IMREAD_GRAYSCALE);

	int** image = (int**)IntAlloc2(img.cols, img.rows);
	*width = img.cols;
	*height = img.rows;

	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			image[i][j] = img.at<unsigned char>(i, j);
		}
	}
	return(image);
}

void Contraction(int** image, int** image_out, int width, int height)
{
	for (int y = 0; y < height - 1; y += 2) {
		for (int x = 0; x < width - 1; x += 2) {
			image_out[y / 2][x / 2] = (image[y][x] + image[y][x + 1] + image[y + 1][x] + image[y + 1][x + 1]) / 4;
		}
	}
}

void IsoM_0(int** img_in, int width, int height, int** img_out)
{
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			img_out[y][x] = img_in[y][x];
		}
	}
}

void IsoM_1(int** img_in, int width, int height, int** img_out)
{
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img_out[i][j] = img_in[i][(width - 1) - j];
		}
	}
}

void IsoM_2(int** img_in, int width, int height, int** img_out)
{
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img_out[i][j] = img_in[(height - 1) - i][j];
		}
	}
}

void IsoM_3(int** img_in, int width, int height, int** img_out)
{
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img_out[i][j] = img_in[j][i];
		}
	}
}

void IsoM_4(int** img_in, int width, int height, int** img_out)
{
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img_out[i][j] = img_in[(height - 1) - j][(width - 1) - i];
		}
	}
}

void IsoM_5(int** img_in, int width, int height, int** img_out)
{
	if (width != height) {
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				img_out[i][j] = img_in[(height - 1) - j][i];
			}
		}
	}
	else {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				img_out[i][j] = img_in[(width - 1) - j][i];
			}
		}
	}
}

void IsoM_6(int** img_in, int width, int height, int** img_out)
{
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img_out[i][j] = img_in[(height - 1) - i][(width - 1) - j];
		}
	}
}

void IsoM_7(int** img_in, int width, int height, int** img_out)
{
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img_out[i][j] = img_in[j][(height - 1) - i];
		}
	}
}

void Isometry(int num, int** img_in, int width, int height, int** img_out)
{
	switch (num)
	{
	case 0:
		IsoM_0(img_in, width, height, img_out); break;
	case 1:
		IsoM_1(img_in, width, height, img_out); break;
	case 2:
		IsoM_2(img_in, width, height, img_out); break;
	case 3:
		IsoM_3(img_in, width, height, img_out); break;
	case 4:
		IsoM_4(img_in, width, height, img_out); break;
	case 5:
		IsoM_5(img_in, width, height, img_out); break;
	case 6:
		IsoM_6(img_in, width, height, img_out); break;
	case 7:
		IsoM_7(img_in, width, height, img_out); break;
	default:
		printf("Isom default", num); break;
	}
}

int ComputeAVG(int** image, int width, int height)
{
	int avg = 0;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			avg += image[j][i];
		}
	}
	return avg = avg / (width * height) + 0.5;
}

void ReadBlock(int** image, int x, int y, int dx, int dy, int** block)
{
	for (int i = 0; i < dy; i++) {
		for (int j = 0; j < dx; j++) {
			block[i][j] = image[y + i][x + j];
		}
	}
}

void WriteBlock(int** image, int x, int y, int dx, int dy, int** block)
{
	for (int i = 0; i < dy; i++) {
		for (int j = 0; j < dx; j++) {
			image[y + i][x + j] = block[i][j];
		}
	}
}

int ComputeAbsError(int** block, int size_block, int** image, int width, int height, int x_temp, int y_temp)
{
	int temp = 0;
	for (int y = 0; y < size_block; y++) {
		for (int x = 0; x < size_block; x++) {
			temp += abs(image[y][x] - block[y + y_temp][x + x_temp]);
		}
	}
	return temp;
}

double ComputeRMSE(int** block, int size_block, int** image, int width, int height, int x_temp, int y_temp)
{
	int mse = 0;
	int n = width * height;
	for (int y = 0; y < size_block; y++) {
		for (int x = 0; x < size_block; x++) {
			int err = image[y][x] - block[y + y_temp][x + x_temp];
			mse += (err * err) / n;
		}
	}
	return sqrt(mse);
}

void Find_AC(int** image, int size_x, int size_y, int block_avg)
{
	for (int y = 0; y < size_y; y++) {
		for (int x = 0; x < size_x; x++) {
			image[y][x] = image[y][x] - block_avg;
		}
	}
}

void Copy_img(int** image, int width, int height, int** img_out) {//�̹��� ����
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			img_out[y][x] = image[y][x];
		}
	}
}

void AC_control(int** image, int width, int height, double alpha, int** temp)
{
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			temp[y][x] = (int)(alpha * image[y][x] + 0.5);
		}
	}
}

bool WriteParameter(const char* name, EncodingResult** A, int x, int y)
{
	FILE* fp = fopen(name, "w");
	if (fp == NULL) {
		printf("\n Failure in fopen!!"); return false;
	}
	for (int ty = 0; ty < y; ty++) {
		for (int tx = 0; tx < x; tx++) {
			fprintf(fp, "%d %d %d %d %f\n", A[ty][tx].x, A[ty][tx].y, A[ty][tx].geo, A[ty][tx].avg, A[ty][tx].alpha);
		}
	}
	fclose(fp);
	return true;
}

bool ReadParameter(const char* name, EncodingResult** A, int width, int height)
{
	FILE* fp = fopen(name, "r");

	if (fp == NULL) {
		printf("\n Failure in fopen!!"); return false;
	}
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			fscanf(fp, "%d%d%d%d%lf", &(A[j][i].x), &(A[j][i].y), &(A[j][i].geo), &(A[j][i].avg), &(A[j][i].alpha));
		}
	}
	fclose(fp);
	return true;
}
