#include <opencv.hpp>
#include <iostream>
#include "usings.hpp"
using namespace cv;
using namespace std;
int main()
{
	/*
	Mat srcImg = imread("E:\\CodeRepositories\\opencv_test\\car\\STOP1.jpg");
	if (srcImg.empty())
		return -1;
	
	point_find(srcImg);
	*/
	/*cv::VideoCapture camera(0);
	if (!camera.isOpened())
		return -1;
	camera.set(CAP_PROP_FRAME_HEIGHT, 1280);
	camera.set(CAP_PROP_FRAME_WIDTH, 720);
	*/
	//多目标检测
	
	//扫码演示
	
	//string turns_info;
	//QRcode(&camera, &turns_info);
	//printf_s("\n%s\n\n", turns_info.c_str());
	

	//轮廓识别演示
	/*
	vector<Point> figure_info;//中心点信息
	vector<RotatedRect> rects;//轮廓矩阵信息
	ColorFinding(&camera, &figure_info, 0);
	*/
	
	//line_detect(&video);

	
	VideoCapture video;
	video.open("E:\\CodeRepositories\\opencv_test\\car\\STOP3.mp4");
	if (!video.isOpened())
		return -1;
	Mat srcImg;
	int index = 2;//图片缩放系数，2^n倍
	vector<Point> centers;
	while (true)
	{
		video >> srcImg;
		if (srcImg.empty())
			break;
		if ((char)waitKey(1) == 27)
			break;
		point_find(srcImg, index, centers);
		
	}
	//TargetTracking(&camera, 1);
	//ellipse_Detect(&camera);
	waitKey(0);
	return 0;
}