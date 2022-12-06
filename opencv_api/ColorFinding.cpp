#include<opencv.hpp>
#include<iostream>
#include<Windows.h>
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
RNG rngs = { 12345 };
void ColorFinding(VideoCapture* camera, vector<Point> *figure_info, vector<RotatedRect> *rects, unsigned RGB)
{
	Mat srcHSV, srcGauss, srcImg;
	int flag = 0;
	while (true)
	{
		Sleep(1000);//1000ms刷新
		flag++;
		*camera >> srcImg;
		if (srcImg.empty())
			return;
		if ((char)waitKey(1) == 27)
		{
			destroyAllWindows();//关闭所有图形窗口
			break;
		}
		if (!(flag < 30))
		{
			destroyAllWindows();//关闭所有图像窗口
			break;
		}
		GaussianBlur(srcImg, srcGauss, Size(7, 7), 0, 0);
		imshow("Gauss", srcGauss);
		cvtColor(srcGauss, srcHSV, COLOR_BGR2HSV);
		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(25, 25));
		inRange(srcHSV, Low[RGB], Up[RGB], srcImg);
		erode(srcImg, srcImg, element);

		vector<vector<Point> >contours;//轮廓容器
		Point2f vtx[4];//矩形顶点容器
		findContours(srcImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//查找轮廓
		Mat dst = Mat::zeros(srcImg.size(), CV_8UC3);//创建空白图像
		for (int i = 0; i < contours.size(); i++) 
		{
			Scalar colors = Scalar(rngs.uniform(0, 255), rngs.uniform(0, 255), rngs.uniform(0, 255)); //创建随机颜色，便于区分
			drawContours(dst, contours, i, colors, 1);//随机颜色绘制轮廓
			RotatedRect rect = minAreaRect(contours[i]);//求解最小矩阵
			rect.points(vtx);//确定旋转矩阵的四个顶点
			cout << "x:" << (vtx[0].x + vtx[2].x) / 2 << endl;
			cout << "y:" << (vtx[0].y + vtx[2].y) / 2 << endl;
			(* figure_info).push_back(Point((vtx[0].x + vtx[2].x) / 2, (vtx[0].y + vtx[2].y) / 2));
			(* rects).push_back(rect);
			for (int j = 0; j < 4;j++)
			{
				line(dst, vtx[j], vtx[(j + 1) % 4], colors, 2);//随机颜色绘制矩形
			}
		}
		imshow("COLOR_Result", dst);
	}
}