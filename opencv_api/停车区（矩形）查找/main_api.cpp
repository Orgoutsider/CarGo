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
	//��Ŀ����
	
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

	
	VideoCapture video;
	video.open("E:\\CodeRepositories\\opencv_test\\car\\STOP3.mp4");
	if (!video.isOpened())
		return -1;
	Mat srcImg;
	int index = 2;//ͼƬ����ϵ����2^n��
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