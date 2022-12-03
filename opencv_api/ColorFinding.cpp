#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;


Scalar R_Low = Scalar(156, 43, 46);
Scalar R_up = Scalar(180, 255, 255);

Scalar G_Low = Scalar(30, 43, 46);
Scalar G_up = Scalar(80, 255, 255);

Scalar B_Low = Scalar(90, 43, 80);
Scalar B_up = Scalar(124, 255, 255);

Scalar Low[3] = { B_Low, G_Low, R_Low };
Scalar Up[3] = { B_up, G_up, R_up };
Mat ColorFinding(Mat srcImg, unsigned RGB)
{
	//flip(srcImg, srcImg, 1);
	Mat srcHSV, srcGauss, temp;
	GaussianBlur(srcImg, srcGauss, Size(7, 7), 0, 0);
	cvtColor(srcGauss, srcHSV, COLOR_BGR2HSV);
	Mat element = getStructuringElement(MORPH_ELLIPSE,Size(25, 25));
	inRange(srcHSV, Low[RGB], Up[RGB], temp);
	erode(temp, temp, element);
	return temp;
}
