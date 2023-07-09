#include <opencv.hpp>
#include <iostream>
double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)//余弦值计算
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}
cv::Point Point_Center(std::vector<cv::Point> approx)//图形中心点计算
{
	double k0 = (double)(approx[2].y - approx[0].y) / (approx[2].x - approx[0].x);
	double k1 = (double)(approx[3].y - approx[1].y) / (approx[3].x - approx[1].x);
	double x_ = (double)(approx[1].y - approx[0].y + k0 * approx[0].x - k1 * approx[1].x) / (k0 - k1); 
	double y_ = (double)(k0 * (x_ - approx[0].x) + approx[0].y);
	return cv::Point(x_, y_);
}
void point_find(cv::Mat srcImage, int index, std::vector<cv::Point>& centers)//输入为原始图片以及存储中心点的向量，index为缩放系数
{
	cv::Mat srcgray, dstImage, resizeImg;
	
#if(1)
	resize(srcImage, srcImage, srcImage.size() / (index/2));
	pyrDown(srcImage, resizeImg, cv::Size(srcImage.cols / 2, srcImage.rows / 2));
#else
	resize(srcImage, resizeImg, srcImage.size() / 4);
#endif
	cvtColor(resizeImg, srcgray, cv::COLOR_BGR2GRAY);//灰度转换
	imshow("gray", srcgray);
	cv::Mat srcbinary;
	threshold(srcgray, srcbinary, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);//阈值化
	imshow("threshold", srcbinary);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(-1, -1));
	morphologyEx(srcbinary, srcbinary, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);//闭操作去除噪点
	imshow("MORPH_OPEN", srcbinary);
	cv::Mat edges;
	Canny(srcbinary, edges, 0, 50, 3, false);//查找边缘
	imshow("edges", edges);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);//只检测最外围的轮廓,只保留拐点的信息
	if (contours.size())
	{
		std::vector<cv::Point> approx;
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);//多边形拟合
			//四边形判断
			if (approx.size() == 4 &&
				fabs(contourArea(approx)) > 1000 &&
				isContourConvex(approx))
			{
				//轮廓角度最大余弦判断
				double maxCosine = 0;

				for (int j = 2; j < 5; j++)
				{
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}
				//矩形判断
				if (maxCosine < 0.3)//轮廓角度的最大余弦，值越小角度越接近90，判断条件越苛刻
				{
					drawContours(resizeImg, contours, i, cv::Scalar(0, 255, 0), 1);//绘制矩形轮廓
					centers.push_back(Point_Center(approx));
					//cout << Point_Center(approx).x << endl;
					//cout << Point_Center(approx).y << endl;
				}
				
			}
			
		}
		for (size_t j = 0; j < centers.size(); j++)
		{
			line(resizeImg, centers[j], centers[j], cv::Scalar(0, 0, 255), 1);//绘制中心点轨迹
		}
	}
	imshow("contours", resizeImg);
}