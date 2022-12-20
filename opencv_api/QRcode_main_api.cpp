#include <opencv.hpp>
#include <iostream>
#include "cv_user.h"
using namespace cv;
using namespace std;
int main()
{
	
	VideoCapture camera(0);
	if (!camera.isOpened())
		return -1;
	camera.set(CAP_PROP_FRAME_HEIGHT, 720);
	camera.set(CAP_PROP_FRAME_WIDTH, 1280);
	
	
	//扫码演示
	
	string turns_info;
	QRcode(&camera, &turns_info);
	printf_s("\n%s\n\n", turns_info.c_str());
	//轮廓识别演示
	
	/*
	vector<Point> figure_info;//中心点信息
	vector<RotatedRect> rects;//轮廓矩阵信息
	ColorFinding(&camera, &figure_info, 2);
	*/
	
	
	/*VideoCapture video;
	video.open("D:\\opencv_test\\chedao.mp4");
	line_detect(&video);
	*/
	waitKey(0);
	return 0;
}