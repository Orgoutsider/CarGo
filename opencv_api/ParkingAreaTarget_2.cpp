#include<opencv.hpp>
#include<iostream>
#include"usings.hpp"
using namespace cv;
using namespace std;

//角度计算
static double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

//矩形绘制
void square_drawing(Mat& Img, vector<vector<Point> > squares, int thickness)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const Point* p = &squares[i][0];

		int n = (int)squares[i].size();
		//dont detect the border
		if (p->x > 3 && p->y > 3)
			polylines(Img, &p, &n, 1, true, Scalar(0, 0, 255), thickness, LINE_AA);
	}
}

//利用灰度信息找矩形
bool square_find(Mat srcImg, vector<vector<Point>>& squares)
{
	Mat srcGray = Saturation(srcImg, 50);
	//灰度处理
	cvtColor(srcGray, srcGray, COLOR_BGR2GRAY);
	Mat srcbinary;
	cv::threshold(srcGray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY_INV);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), cv::Point(-1, -1));
	morphologyEx(srcbinary, srcbinary, MORPH_CLOSE, kernel, cv::Point(-1, -1), 2); // 闭操作去除噪点
	morphologyEx(srcbinary, srcbinary, MORPH_OPEN, kernel, cv::Point(-1, -1), 2); // 开操作去除缺口
	//添加边框
	Mat FImg = cv::Mat(srcbinary.size(), CV_8UC1, cv::Scalar::all(0));
	int rows = FImg.rows;
	int cols = FImg.cols;
	for (size_t i = 0; i < rows; i++)// 行处理
	{
		for (size_t j = 0; j < 1; j++)
		{
			FImg.at<uchar>(i, j) = 255;
			FImg.at<uchar>(i, cols - 1 - j) = 255;
		}
	}
	for (size_t i = 0; i < cols; i++)// 列处理
	{
		for (size_t j = 0; j < 1; j++)
		{
			FImg.at<uchar>(j, i) = 255;
			FImg.at<uchar>(rows - 1 - j, i) = 255;
		}
	}
	bitwise_or(FImg, srcbinary, srcbinary);

	Mat edges;
	Canny(srcbinary, edges, 0, 50, 3, false);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<Vec4i> hierarchy;
	// 检测所有轮廓,只保留拐点的信息
	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); 
	vector<Point> approx;
	if (contours.size())
	{
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.03, true);
			//调试的时候把轮廓面积判断改成了3000
			if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 3000 && isContourConvex(Mat(approx)))
			{
				// find the maximum cosine of the angle between joint edges
				double maxCosine = 0;
				for (int j = 2; j < 5; j++)
				{
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}
				// if cosines of all angles are small
				// (all angles are ~90 degree) then write quandrange
				// vertices to resultant sequence
				if (maxCosine < 0.4)
					squares.push_back(approx);
			}
		}
	}
	if (squares.size())
		return true;
	else
		return false;
}

//删除多余点
void square_point_delete(vector<Point>& approx)
{
	int approxSize = approx.size();
	while (approxSize > 4)
	{
		//定义边长结构体，lenth存放长度，point存放端点标号
		struct _side_length
		{
			vector<int> lenth;
			vector<Point2i> point;
		}side_length;
		//计算边长长度
		for (size_t i = 0; i < approxSize; i++)
		{
			side_length.lenth.push_back(sqrt(pow((approx[i].x - approx[(i + 1) % approxSize].x), 2) + pow((approx[i].y - approx[(i + 1) % approxSize].y), 2)));
			side_length.point.push_back(Point(i, (i + 1) % approxSize));
		}
		//寻找最短边长的标号
		int lenth_temp = side_length.lenth[0];
		int lenth_flag = 0;//最短边长的标号
		int point_flag = 0;//目标点的标号
		for (size_t i = 0; i < approxSize; i++)
		{
			if (lenth_temp > side_length.lenth[i])
			{
				lenth_flag = i;
				lenth_temp = side_length.lenth[i];
			}
		}
		//比较最短边前一条边和后一条边的长度
		int front_side = (lenth_flag - 1) < 0 ? (approxSize - 1) : (lenth_flag - 1);//前一条边的序号
		int back_side = (lenth_flag + 1) > (approxSize - 1) ? 0 : (lenth_flag + 1);//后一条边的序号
		//得到目标点的标号
		point_flag = 
			  side_length.lenth[front_side]
			< side_length.lenth[back_side]
			? side_length.point[lenth_flag].x
			: side_length.point[lenth_flag].y;
		//删除目标点
		approx.erase(approx.begin() + point_flag);
		approxSize--;
	}
}

// 使用颜色信息找矩形
// 如果找不到，就找出最大的轮廓，并进行删点操作，最后得到四边形
bool square_find_color(Mat srcImg, vector<vector<Point> >& squares)
{
	//Low of S can be adjusted. High of S and V must be set to 255.
	Scalar low_Area_Color = Scalar(70, 20, 10);
	Scalar high_Area_Color = Scalar(145, 255, 255);

	Mat srcHSV;
	cvtColor(srcImg, srcHSV, COLOR_BGR2HSV);
	Mat StopArea;
	inRange(srcHSV, low_Area_Color, high_Area_Color, StopArea);
	//imshow("HSV", StopArea);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<Vec4i> hierarchy;
	// 检测最外围轮廓,只保留拐点的信息
	findContours(StopArea, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	vector<Point> approx;
	if (contours.size())
	{
		//找矩形
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.03, true);
			if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)))
			{
				double maxCosine = 0;
				for (int j = 2; j < 5; j++)
				{
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}
				if (maxCosine < 0.4)
					squares.push_back(approx);
			}
		}
	}
	if (squares.size())
		return true;
	else
	{
		//找到最大的轮廓
		if (contours.size())
		{
			double countersArea_temp = contourArea(contours[0]);
			int MaxAreaNum = 0;
			for (size_t i = 1; i < contours.size(); i++)
			{
				if (countersArea_temp < contourArea(contours[i]))
				{
					countersArea_temp = contourArea(contours[i]);
					MaxAreaNum = i;
				}
			}
			//多边形拟合
			vector<Point> approx;
			approxPolyDP(Mat(contours[MaxAreaNum]), approx, arcLength(Mat(contours[MaxAreaNum]), true) * 0.03, true);
			square_point_delete(approx);
			squares.push_back(approx);
		}
		return true;
	}
	return true;
}

void ParkingAreaTarget(Mat srcImg)
{
	//滤波处理
	resize(srcImg, srcImg, Size(srcImg.cols / 4, srcImg.rows / 4));
	GaussianBlur(srcImg, srcImg, Size(3, 3), 0, 0);

	//设置矩形容器并寻找矩形
	vector<vector<Point>> squares;
	if (square_find(srcImg, squares))
	{
		putText(srcImg, "square find by GRAY", Point(30, 50), FONT_HERSHEY_PLAIN, 3, Scalar(200, 200, 50), 2, 8);
		//cout << "square find by GRAY" << endl;
		square_drawing(srcImg, squares, 1);
	}
	else if(square_find_color(srcImg, squares))
	{
		putText(srcImg, "square find by COLOR", Point(30, 50), FONT_HERSHEY_PLAIN, 3, Scalar(50, 200, 200), 2, 8);
		//cout << "square find by COLOR" << endl;
		square_drawing(srcImg, squares, 1);
	}
	imshow("srcImg", srcImg);
}


void ParkingAreaTarget_()
{
	int name_c = 9;
	vector<String> Img_names;
	const char* name_1 = "E:\\CodeRepositories\\opencv_test\\car\\light\\light";
	const char* name_2 = ".jpg";

	for (int i = 1; i <= 13; i++)
	{
		char name[100] = {};
		sprintf_s(name, "%s%d%s", name_1, i, name_2);
		Img_names.push_back(name);
	}

	Mat Img = imread(Img_names[name_c]);
	if (Img.empty())
		return;
	//imshow("origin", Img);
	ParkingAreaTarget(Img);
}
