#include<opencv.hpp>
#include<iostream>
#include <uchar.h>
//边框线宽
int line_width = 1;
void SquareWithBorder(cv::Mat& srcImage)
{
	if (srcImage.type() != CV_8UC1)
	{
		srcImage.convertTo(srcImage, CV_8UC1);
	}
	cv::Mat FImg = cv::Mat(srcImage.size(), CV_8UC1, cv::Scalar::all(0));
	int rows = FImg.rows;
	int cols = FImg.cols;
	//行处理
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < line_width; j++)
		{
			FImg.at<uchar>(i, j) = 255;
			FImg.at<uchar>(i, cols - 1 - j) = 255;
		}
	}
	//列处理
	for (size_t i = 0; i < cols; i++)
	{
		for (size_t j = 0; j < line_width; j++)
		{
			FImg.at<uchar>(j, i) = 255;
			FImg.at<uchar>(rows - 1 - j, i) = 255;
		}
		
	}
	cv::imshow("Fimg", FImg);
	cv::bitwise_or(FImg, srcImage, srcImage);

}