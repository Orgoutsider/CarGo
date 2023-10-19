#include<opencv.hpp>
#include<iostream>
#include"usings.hpp"


//角度计算
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

//矩形绘制
void square_drawing(cv::Mat& Img, std::vector<std::vector<cv::Point> > squares, int thickness)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const cv::Point* p = &squares[i][0];

		int n = (int)squares[i].size();
		//dont detect the border
		if (p->x > 3 && p->y > 3)
			polylines(Img, &p, &n, 1, true, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA);
	}
}

//利用灰度信息找矩形
bool square_find(cv::Mat srcImg, std::vector<std::vector<cv::Point>>& squares)
{
	cv::Mat srcGray = Saturation(srcImg, 50);
	//灰度处理
	cv::cvtColor(srcGray, srcGray, cv::COLOR_BGR2GRAY);
	cv::Mat srcbinary;
	cv::threshold(srcGray, srcbinary, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY_INV);
	cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
	morphologyEx(srcbinary, srcbinary, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2); // 闭操作去除噪点
	morphologyEx(srcbinary, srcbinary, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2); // 开操作去除缺口
	//添加边框
	cv::Mat FImg = cv::Mat(srcbinary.size(), CV_8UC1, cv::Scalar::all(0));
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

	cv::Mat edges;
	Canny(srcbinary, edges, 0, 50, 3, false);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	// 检测所有轮廓,只保留拐点的信息
	findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Point> approx;
	if (contours.size())
	{
		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.03, true);
			//调试的时候把轮廓面积判断改成了3000
			if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 3000 && cv::isContourConvex(cv::Mat(approx)))
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
void square_point_delete(std::vector<cv::Point>& approx)
{
	int approxSize = approx.size();
	while (approxSize > 4)
	{
		//定义边长结构体，lenth存放长度，point存放端点标号
		struct _side_length
		{
			std::vector<int> lenth;
			std::vector<cv::Point2i> point;
		}side_length;
		//计算边长长度
		for (size_t i = 0; i < approxSize; i++)
		{
			side_length.lenth.push_back(sqrt(pow((approx[i].x - approx[(i + 1) % approxSize].x), 2) + pow((approx[i].y - approx[(i + 1) % approxSize].y), 2)));
			side_length.point.push_back(cv::Point(i, (i + 1) % approxSize));
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
bool square_find_color(cv::Mat srcImg, std::vector<std::vector<cv::Point>>& squares)
{
	//Low of S can be adjusted. High of S and V must be set to 255.
	cv::Scalar low_Area_Color = cv::Scalar(70, 20, 10);
	cv::Scalar high_Area_Color = cv::Scalar(145, 255, 255);

	cv::Mat srcHSV;
	cv::cvtColor(srcImg, srcHSV, cv::COLOR_BGR2HSV);
	cv::Mat StopArea;
	cv::inRange(srcHSV, low_Area_Color, high_Area_Color, StopArea);
	//imshow("HSV", StopArea);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	// 检测最外围轮廓,只保留拐点的信息
	findContours(StopArea, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Point> approx;
	if (contours.size())
	{
		//找矩形
		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.03, true);
			if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx)))
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
			std::vector<cv::Point> approx;
			cv::approxPolyDP(cv::Mat(contours[MaxAreaNum]), approx, cv::arcLength(cv::Mat(contours[MaxAreaNum]), true) * 0.03, true);
			square_point_delete(approx);
			squares.push_back(approx);
		}
		return true;
	}
	return true;
}

void ParkingAreaTarget(cv::Mat srcImg)
{
	//滤波处理
	cv::resize(srcImg, srcImg, cv::Size(srcImg.cols / 4, srcImg.rows / 4));
	cv::GaussianBlur(srcImg, srcImg, cv::Size(3, 3), 0, 0);

	//设置矩形容器并寻找矩形
	std::vector<std::vector<cv::Point>> squares;
	if (square_find(srcImg, squares))
	{
		cv::putText(srcImg, "square find by GRAY", cv::Point(30, 50), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(200, 200, 50), 2, 8);
		//cout << "square find by GRAY" << endl;
		square_drawing(srcImg, squares, 1);
	}
	else if(square_find_color(srcImg, squares))
	{
		cv::putText(srcImg, "square find by COLOR", cv::Point(30, 50), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(50, 200, 200), 2, 8);
		//cout << "square find by COLOR" << endl;
		square_drawing(srcImg, squares, 1);
	}
	imshow("srcImg", srcImg);
}


void ParkingAreaTarget_()
{
	int name_c = 1;
	std::vector<cv::String> Img_names;
	const char* name_1 = "E:\\CodeRepositories\\opencv_test\\car\\light\\light";
	const char* name_2 = ".jpg";

	for (int i = 1; i <= 13; i++)
	{
		char name[100] = {};
		sprintf_s(name, "%s%d%s", name_1, i, name_2);
		Img_names.push_back(name);
	}

	cv::Mat Img = cv::imread(Img_names[name_c]);
	if (Img.empty())
		return;
	//imshow("origin", Img);
	ParkingAreaTarget(Img);
}
