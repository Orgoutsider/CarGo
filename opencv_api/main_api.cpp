#include <opencv.hpp>
#include <iostream>
#include <fstream>
#include "QRcode.h"
using namespace cv;
using namespace std;
int main()
{
	VideoCapture camera(0);
	if (!camera.isOpened())
		return -1;
	camera.set(CAP_PROP_FRAME_HEIGHT, 720);
	camera.set(CAP_PROP_FRAME_WIDTH, 1280);
	string turns_info;
	QRcode(&camera, &turns_info);
	printf_s("\n%s\n", turns_info.c_str());
	waitKey(0);
	return 0;
}