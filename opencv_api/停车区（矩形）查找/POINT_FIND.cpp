
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
RNG rng[123456];
double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}
Point Point_Center(vector<Point> approx)
{
	double k0 = (double)(approx[2].y - approx[0].y) / (approx[2].x - approx[0].x);
	double k1 = (double)(approx[3].y - approx[1].y) / (approx[3].x - approx[1].x);
	double x_ = (double)(approx[1].y - approx[0].y + k0 * approx[0].x - k1 * approx[1].x) / (k0 - k1);
	double y_ = (double)(k0 * (x_ - approx[0].x) + approx[0].y);
	return Point(x_, y_);
}
void point_find(Mat srcImage, int index, vector<Point>& centers)
{
	Mat srcgray, dstImage, resizeImg;
	
#if(1)
	resize(srcImage, srcImage, srcImage.size() / (index/2));
	pyrDown(srcImage, resizeImg, Size(srcImage.cols / 2, srcImage.rows / 2));
#else
	resize(srcImage, resizeImg, srcImage.size() / 4);
#endif
	cvtColor(resizeImg, srcgray, COLOR_BGR2GRAY);
	imshow("gray", srcgray);
	Mat srcbinary;
	threshold(srcgray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY);
	imshow("threshold", srcbinary);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
	morphologyEx(srcbinary, srcbinary, MORPH_CLOSE, kernel, Point(-1, -1));
	imshow("MORPH_OPEN", srcbinary);
	Mat edges;
	Canny(srcbinary, edges, 0, 50, 3, false);
	imshow("edges", edges);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//只检测最外围的轮廓,只保留拐点的信息
	if (contours.size())
	{
		vector<Point> approx;
		//int count = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);
			if (approx.size() == 4 &&
				fabs(contourArea(approx)) > 1000 &&
				isContourConvex(approx))
			{
				double maxCosine = 0;

				for (int j = 2; j < 5; j++)
				{
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}
				if (maxCosine < 0.3)
				{
					//count++;
					drawContours(resizeImg, contours, i, Scalar(0, 255, 0), 1);
					centers.push_back(Point_Center(approx));
					//cout << Point_Center(approx).x << endl;
					//cout << Point_Center(approx).y << endl;
				}
				
			}
			
		}
		for (size_t j = 0; j < centers.size(); j++)
		{
			line(resizeImg, centers[j], centers[j], Scalar(0, 0, 255), 1);
		}
		//cout << count << endl;
	}
	imshow("contours", resizeImg);
}