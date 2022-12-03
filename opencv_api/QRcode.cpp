#include <opencv.hpp>
#include <iostream>
#include <fstream>
#include "QRcode.h"
using namespace cv;

void QRcode(VideoCapture *camera, std::string* info)
{
	Mat QR;
	while (true)
	{
		*camera >> QR;
		if (!QR.data)
			return;
		Mat CODE = QR.clone();
		Mat result = QR.clone();
		cvtColor(CODE, CODE, COLOR_BGR2GRAY);
		Mat straight_qrcode;
		QRCodeDetector QR_detector;
		std::vector<Point> points;
		*info = QR_detector.detectAndDecode(CODE, points, straight_qrcode);
		if (!( * info).empty())
		{
			putText(result, *info, Point(20, 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 255), 2, 8);
			imshow("res", result);
			break;
		}
		points.clear();
	}
}
