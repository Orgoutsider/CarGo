#include<opencv.hpp>
#include<iostream>
#include "cv_user.h"
using namespace cv;
using namespace std;
int main()
{
	/*
	cv::VideoCapture camera(0);
	if (!camera.isOpened())
		return -1;
	camera.set(CAP_PROP_FRAME_HEIGHT, 1920);
	camera.set(CAP_PROP_FRAME_WIDTH, 1080);
	*/

	VideoCapture video;
	video.open("E:\\opencv_test\\car\\VID4.mp4");
	ellipseTargetFind(video);
	waitKey(0);
	return 0;
}