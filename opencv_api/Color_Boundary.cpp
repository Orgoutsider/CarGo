#include<opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//颜色增强函数相关宏定义
#define max2(a,b) (a>b?a:b)
#define max3(a,b,c) (a>b?max2(a,c):max2(b,c))
#define min2(a,b) (a<b?a:b)
#define min3(a,b,c) (a<b?min2(a,c):min2(b,c))

cv::Mat Saturation(cv::Mat src, int percent);//颜色增强函数
cv::Mat LBD_Color_Func(cv::Mat Enhanced);

void LBD_ColorEnhanced(Mat Img)
{
	resize(Img, Img, Size(Img.cols / 4, Img.rows / 4));//缩放图像
	imshow("original", Img);

	cv::Mat Enhanced = Saturation(Img, 50);//饱和度调整参数，-100 — 100, 正数饱和度增强，负数饱和度减弱
	//imshow("Enhanced", Enhanced);
	GaussianBlur(Enhanced, Enhanced, Size(3, 3), 0, 0);//滤波预处理
	Mat ColorImg = LBD_Color_Func(Enhanced);
	imshow("LineImg", ColorImg);
}

//颜色增强函数
cv::Mat Saturation(cv::Mat src, int percent)
{
	float Increment = percent * 1.0f / 100;
	cv::Mat temp = src.clone();
	int row = src.rows;
	int col = src.cols;
	for (int i = 0; i < row; ++i)
	{
		uchar* t = temp.ptr<uchar>(i);
		uchar* s = src.ptr<uchar>(i);
		for (int j = 0; j < col; ++j)
		{
			uchar b = s[3 * j];
			uchar g = s[3 * j + 1];
			uchar r = s[3 * j + 2];
			float max = max3(r, g, b);
			float min = min3(r, g, b);
			float delta, value;
			float L, S, alpha;
			delta = (max - min) / 255;
			if (delta == 0)
				continue;
			value = (max + min) / 255;
			L = value / 2;
			if (L < 0.5)
				S = delta / value;
			else
				S = delta / (2 - value);
			if (Increment >= 0)
			{
				if ((Increment + S) >= 1)
					alpha = S;
				else
					alpha = 1 - Increment;
				alpha = 1 / alpha - 1;
				t[3 * j + 2] = static_cast<uchar>(r + (r - L * 255) * alpha);
				t[3 * j + 1] = static_cast<uchar>(g + (g - L * 255) * alpha);
				t[3 * j] = static_cast<uchar>(b + (b - L * 255) * alpha);
			}
			else
			{
				alpha = Increment;
				t[3 * j + 2] = static_cast<uchar>(L * 255 + (r - L * 255) * (1 + alpha));
				t[3 * j + 1] = static_cast<uchar>(L * 255 + (g - L * 255) * (1 + alpha));
				t[3 * j] = static_cast<uchar>(L * 255 + (b - L * 255) * (1 + alpha));
			}
		}
	}
	return temp;
}

cv::Mat LBD_Color_Func(cv::Mat Enhanced)
{
	Mat HSVImg;
	cvtColor(Enhanced, HSVImg, COLOR_BGR2HSV);
	imshow("HSVImg", HSVImg);

	Scalar Boundary_low = Scalar(0, 0, 160);
	Scalar Boundary_high = Scalar(179, 43, 255);
	Mat RangeImg;
	inRange(HSVImg, Boundary_low, Boundary_high, RangeImg);
	imshow("RangeImg", RangeImg);

	Mat edgeImg;
	Canny(RangeImg, edgeImg, 50, 200, 3);
	imshow("edge", edgeImg);

	Mat LineImg;
	cvtColor(edgeImg, LineImg, COLOR_GRAY2BGR);
	vector<Vec2f> lines;
	HoughLines(edgeImg, lines, 1, CV_PI / 180, 200, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(Enhanced, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
	}
	return Enhanced;

}