#include <opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;
//先找到物体矩形框，我想的是把找到的矩形框传递到轨迹跟踪函数中，这里我借用的是颜色识别的那一套
Scalar R_Low = Scalar(156, 100, 80);
Scalar R_up = Scalar(180, 255, 255);

Scalar G_Low = Scalar(30, 50, 46);
Scalar G_up = Scalar(85, 255, 255);

Scalar B_Low = Scalar(85, 100, 100);
Scalar B_up = Scalar(120, 255, 255);

Scalar Low[3] = { B_Low, G_Low, R_Low };
Scalar Up[3] = { B_up, G_up, R_up };

Mat rectImg;//物体图像框
Mat targetImgHSV;//物体图像HSV
int histSize = 200;
float histR[] = { 0,255 };
const float* histRange = histR;
int channels[] = { 0,1 };
Mat dstHist;
vector<Point> pt; //保存目标轨迹

bool TargetFinding(Mat srcImg, Rect *rect_, unsigned COLOR)
{
	Mat srcF;
	//resize(srcImg, srcImg, srcImg.size() / 4);
	GaussianBlur(srcImg, srcF, Size(5, 5), 0, 0);
	cvtColor(srcF, srcF, COLOR_BGR2HSV);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(13, 13));
	inRange(srcF, Low[COLOR], Up[COLOR], srcF);
	erode(srcF, srcF, element);
	vector<vector<Point> >contours;//轮廓容器
	findContours(srcF, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//查找轮廓
	if (!contours.empty())
	{
		*rect_ = boundingRect(contours[0]);//求解矩阵
		float Rect_area = (* rect_).area();
		int tag = 0;//目标最大矩阵标志位
		for (int i = 1; i < contours.size(); i++)
		{
			Rect rect_temp = boundingRect(contours[i]);//求解矩阵
			float Rect_area_temp = rect_temp.area();
			if (Rect_area_temp > Rect_area)
			{
				tag = i;
				*rect_ = rect_temp;
				Rect_area = Rect_area_temp;
			}
		}
		return true;
	}
	else return false;
}
void TargetTracking(VideoCapture* camera, unsigned RGB)
{
	Mat srcImg;
	Rect rect;
	bool targetFound = false;
	while (!targetFound)
	{
		*camera >> srcImg;
		targetFound = TargetFinding(srcImg, &rect, RGB);
		if(targetFound)
		imshow("rect", srcImg(rect));
	}
	rectImg = srcImg(rect);
	imshow("target", rectImg);
	cvtColor(rectImg, targetImgHSV, COLOR_RGB2HSV);
	calcHist(&targetImgHSV, 2, channels, Mat(), dstHist, 1, &histSize, &histRange, true, false);
	normalize(dstHist, dstHist, 0, 255, NORM_MINMAX);
	while (true)
	{
		*camera >> srcImg;
		if (srcImg.empty())
			break;
		if ((char)waitKey(1) == 27)
			break;
		Mat imageHSV;
		Mat calcBackImage;
		cvtColor(srcImg, imageHSV, COLOR_BGR2HSV);
		calcBackProject(&imageHSV, 2, channels, dstHist, calcBackImage, &histRange); //反向投影
		TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001);
		CamShift(calcBackImage, rect, criteria);
		Mat imageROI = imageHSV(rect); //更新模板
		targetImgHSV = imageHSV(rect);
		calcHist(&imageROI, 2, channels, Mat(), dstHist, 1, &histSize, &histRange);
		normalize(dstHist, dstHist, 0.0, 1.0, NORM_MINMAX); //归一化
		rectangle(srcImg, rect, Scalar(255, 0, 0), 3); //目标绘制
		pt.push_back(Point(rect.x + rect.width / 2, rect.y + rect.height / 2));
		for (int i = 0; i < pt.size() - 1; i++)
		{
			line(srcImg, pt[i], pt[i + 1], Scalar(0, 255, 0), 2.5);
		}
		imshow("Result", srcImg);
	}
}
