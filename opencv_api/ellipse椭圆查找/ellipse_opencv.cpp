#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;

const int Gauss_size = 3;//高斯平滑内核大小
const int Canny_low = 50;//第一次Canny边缘查找的第一滞后因子
const int Canny_up = 100;//第一次Canny边缘查找的第二滞后因子

const int con_Area_min = 4500;//粗筛-最小面积阈值
const int con_Point_cont = 20;//粗筛-图形最少点个数阈值，即连成某个封闭轮廓的点的个数，少于该阈值表明轮廓无效
const int con_Area_max = 200000;//粗筛-最大面积阈值

//直线斜率处处相等原理的相关参数
const int line_Point_1 = 10;//点1，该数字表示围成封闭轮廓点的序号，只要不太离谱即可
const int line_Point_2 = 20;//点2
const int line_threshold = 0.5;//判定阈值，小于即判定为直线

bool flag[100];//聚类标识位

//椭圆圆心十字光标绘制，用于调试观察
void drawCross(Mat& img, Point2f point, Scalar color, int size, int thickness = 1)
{
	//绘制横线
	line(img, Point(point.x - size / 2, point.y), Point(point.x + size / 2, point.y), color, thickness, 8, 0);
	//绘制竖线
	line(img, Point(point.x, point.y - size / 2), Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

//标识位初始化
void flag_init()
{
	for (size_t i = 0; i < 100; i++)
	{
		flag[i] = false;
	}
}
void ellipseTargetFind(VideoCapture camera)
{
	Mat srcImg;
	while (true)
	{
		camera >> srcImg;
		if (srcImg.empty())
			break;
		if ((char)waitKey(1) == 27)
			break;
		
		Mat srcdst, srcCopy;//从相机传进来需要两张图片
		Point2f _center;//椭圆中心
		vector<Point2f> centers;//椭圆中心容器
		vector<Point2f> center;//目标椭圆容器
		resize(srcImg, srcdst, srcImg.size());//重设大小，可选
		srcCopy = srcdst.clone();

		//第一次预处理
		GaussianBlur(srcdst, srcdst, Size(Gauss_size, Gauss_size), 0, 0);
		cvtColor(srcdst, srcdst, COLOR_BGR2GRAY);
		Canny(srcdst, srcdst, Canny_low, Canny_up, 3);
		//imshow("step1.", srcdst);//用于调试
		//ROI设置
		Mat mm = srcCopy(Rect(0, 0, srcCopy.cols, srcCopy.rows));
		mm = { Scalar(0, 0, 0) };//把ROI中的像素值改为黑色

		//第一次轮廓查找
		vector<vector<Point> > contours;// 创建容器，存储轮廓
		vector<Vec4i> hierarchy;// 寻找轮廓所需参数
		findContours(srcdst, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

		//Mat imageContours = Mat::zeros(mm.size(), CV_8UC1);//创建轮廓展示图像，用于调试
		//如果查找到了轮廓
		if (contours.size())
		{
			
			//轮廓展示，用于调试
			/*
			for (int i = 0; i < contours.size(); i++)
			{
				drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
			}
			imshow("Contours_1", imageContours);
			*/
			//第一次排除
			for (int i = 0; i < contours.size(); i++)
			{
				//初筛
				if (contourArea(contours[i]) < con_Area_min || contours[i].size() < con_Point_cont || contourArea(contours[i]) > con_Area_max)
					continue;
				//利用直线斜率处处相等的原理
				if (abs(((double)(contours[i][0].y - contours[i][line_Point_1].y) / (double)(contours[i][0].x - contours[i][line_Point_1].x) - (double)(contours[i][line_Point_1].y - contours[i][line_Point_2].y) / (double)(contours[i][line_Point_1].x - contours[i][line_Point_2].x))) < line_threshold)
					continue;
				//利用凹凸性的原理
				if (!abs((contours[i][0].y + contours[i][20].y) / 2 - contours[i][10].y))
					continue;
				//drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
				RotatedRect m_ellipsetemp;//创建接收椭圆的容器
				m_ellipsetemp = fitEllipse(contours[i]);  //找到的第一个轮廓，放置到m_ellipsetemp
				if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.2)
					continue;
				ellipse(mm, m_ellipsetemp, cv::Scalar(255, 255, 255));//在图像中绘制椭圆，必要
			}
			//imshow("Contours_1", imageContours);
			imshow("mm", mm);//显示第一次排除结果，用于调试
		}
		//第二次预处理
		Mat mmdst;
		GaussianBlur(mm, mmdst, Size(Gauss_size, Gauss_size), 0, 0);
		cvtColor(mmdst, mmdst, COLOR_BGR2GRAY);
		Canny(mmdst, mmdst, 50, 150, 3);
		//imshow("step2.", mmdst);//用于调试
		//第二次轮廓查找
		vector<vector<Point> > contours1;// 创建容器，存储轮廓
		vector<Vec4i> hierarchy1;// 寻找轮廓所需参数
		findContours(mmdst, contours1, hierarchy1, RETR_CCOMP, CHAIN_APPROX_NONE);
		//Mat imageContours1 = Mat::zeros(mmdst.size(), CV_8UC1);//创建轮廓展示图像，用于调试
		//如果查找到了轮廓
		if (contours1.size())
		{
			//第二次筛除
			for (int i = 0; i < contours1.size(); i++)
			{
				//初筛
				if (contours1[i].size() < 10 || contours1[i].size() > 1000)
					continue;
				//利用凹凸性的原理
				//if (!abs((contours1[i][0].y + contours1[i][20].y) / 2 - contours1[i][10].y))
				//	continue;
				if (contourArea(contours1[i]) < 4500 || contourArea(contours1[i]) > 180000)
					continue;
				//drawContours(imageContours1, contours1, i, Scalar(255), 1, 8);//用于调试

				RotatedRect m_ellipsetemp1;
				m_ellipsetemp1 = fitEllipse(contours1[i]);
				ellipse(srcImg, m_ellipsetemp1, cv::Scalar(255, 0, 0));//绘制椭圆，用于调试
				_center = m_ellipsetemp1.center;//读取椭圆中心，必要
				//drawCross(srcCopy, _center, Scalar(255, 0, 0), 30, 2);//绘制中心十字，用于调试
				circle(srcImg, _center, 1, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）, 用于调试
				centers.push_back(_center);
			}
			//imshow("Contours_2", imageContours1);//用于调试
			//cout << centers.size() << endl;
			//聚类
			flag_init();
			for (int i = 0; i < centers.size() - 2; i++)
			{
				flag[i] = true;
				int x_temp = centers[i].x, y_temp = centers[i].y, count = 1;
				for (int j = 1; j < centers.size(); j++)
				{
					if (abs(centers[i].x - centers[j].x) < 10 && abs(centers[i].y - centers[j].y) < 10)
					{
						if (!flag[j])
						{
							flag[j] = true;
							x_temp = x_temp + centers[j].x;
							y_temp = y_temp + centers[j].y;
							count++;
						}
						else continue;
					}
				}
				if (count > 2)
				{
				//平均数求聚类中心，感觉不太妥当，但是精度感觉还行，追求精度的话可以用 Weiszfeld 算法求中位中心，那个要迭代
				center.push_back(Point((int)x_temp / count, (int)y_temp / count));
				}
			}
			cout << "椭圆个数： " << center.size() << endl;
			//绘制中心十字，用于调试
			for (size_t i = 0; i < center.size(); i++)
			{
				drawCross(srcImg, center[i], Scalar(0, 0, 255), 30, 2);
			}
			//颜色标定
			//目前就差这一步了
		}
		imshow("srcCopy", srcImg);//用于调试
	}
}