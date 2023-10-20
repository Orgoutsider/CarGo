#include<opencv.hpp>
#include<iostream>

//五边形补充多余点
void square_point_add(std::vector<cv::Point>& approx)
{
	int approxSize = approx.size();
	//lenth存放长度，point存放端点标号
	std::vector<int> lenth;
	std::vector<cv::Point2i> point;

	//计算边长长度
	for (size_t i = 0; i < approxSize; i++)
	{
		lenth.push_back(sqrt(pow((approx[i].x - approx[(i + 1) % approxSize].x), 2)
			+ pow((approx[i].y - approx[(i + 1) % approxSize].y), 2)));
		point.push_back(cv::Point(i, (i + 1) % approxSize));
	}
	//寻找最短边长的标号
	int lenth_temp = lenth[0];
	int lenth_flag = 0;//最短边长的标号
	for (size_t i = 0; i < approxSize; i++)
	{
		if (lenth_temp > lenth[i])
		{
			lenth_flag = i;
			lenth_temp = lenth[i];
		}
	}

	int front_side = (lenth_flag - 1) < 0 ? (approxSize - 1) : (lenth_flag - 1);//前一条边的序号
	int back_side = (lenth_flag + 1) > (approxSize - 1) ? 0 : (lenth_flag + 1);//后一条边的序号
	cv::Point P1 = approx[point[front_side].x];
	cv::Point P2 = approx[point[lenth_flag].x];
	cv::Point P3 = approx[point[lenth_flag].y];
	cv::Point P4 = approx[point[back_side].y];
	cv::Point NewPoint;

	double k1 = (double)(P2.y - P1.y) / (P2.x - P1.x);
	double k2 = (double)(P4.y - P3.y) / (P4.x - P3.x);
	NewPoint.x = (double)(k1 * P1.x - k2 * P3.x + P3.y - P1.y) / (k1 - k2);
	NewPoint.y = (double)k1 * (NewPoint.x - P1.x) + P1.y;

	if (point[lenth_flag].x == approxSize - 1)
	{
		approx.erase(approx.begin() + point[lenth_flag].x);
		approx.erase(approx.begin());
		approx.insert(approx.begin(), NewPoint);
	}
	else
	{
		approx.erase(approx.begin() + point[lenth_flag].x);
		approx.erase(approx.begin() + point[lenth_flag].x);
		approx.insert(approx.begin() + point[lenth_flag].x, NewPoint);
	}
}