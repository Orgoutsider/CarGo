#include <opencv.hpp>
#include <iostream>
#include "cv_user.h"
using namespace cv;
using namespace std;
int main()
{
	cv::VideoCapture camera(0);
	if (!camera.isOpened())
		return -1;
	camera.set(CAP_PROP_FRAME_HEIGHT, 1280);
	camera.set(CAP_PROP_FRAME_WIDTH, 720);
	
	
	//ɨ����ʾ
	
	//string turns_info;
	//QRcode(&camera, &turns_info);
	//printf_s("\n%s\n\n", turns_info.c_str());
	

	//����ʶ����ʾ
	/*
	vector<Point> figure_info;//���ĵ���Ϣ
	vector<RotatedRect> rects;//����������Ϣ
	ColorFinding(&camera, &figure_info, 0);
	*/
	
	//line_detect(&video);

	/*
	VideoCapture video;
	video.open("E:\\opencv_test\\car\\VID.mp4");
	if (!video.isOpened())
		return -1;
	*/
	TargetTracking(&camera, 0);
	waitKey(0);
	return 0;
}