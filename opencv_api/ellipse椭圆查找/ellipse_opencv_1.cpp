#include<opencv.hpp>
#include<iostream>
#include<stdlib.h>
using namespace cv;
using namespace std;

const int Gauss_size = 3;//高斯平滑内核大小
const int Canny_low = 50;//第一次Canny边缘查找的第一滞后因子
const int Canny_up = 100;//第一次Canny边缘查找的第二滞后因子

const int con_Area_min = 3000;//粗筛-最小面积阈值
const int con_Point_cont = 15;//粗筛-图形最少点个数阈值，即连成某个封闭轮廓的点的个数，少于该阈值表明轮廓无效
const int con_Area_max = 150000;//粗筛-最大面积阈值

//直线斜率处处相等原理的相关参数
const int line_Point_1 = 10;//点1，该数字表示围成封闭轮廓点的序号，只要不太离谱即可
const int line_Point_2 = 20;//点2
const int line_threshold = 0.5;//判定阈值，小于即判定为直线

bool jvlei_flag[100];//聚类标识位
bool color_flag[100];//颜色标记标识位

//椭圆圆心十字光标绘制，用于调试观察
void drawCross(Mat& img, Point2f point, Scalar color, int size, int thickness = 1)
{
	//绘制横线
	line(img, Point(point.x - size / 2, point.y), Point(point.x + size / 2, point.y), color, thickness, 8, 0);
	//绘制竖线
	line(img, Point(point.x, point.y - size / 2), Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

//标识位初始化
void flag_init(bool* _FLAG)
{
	for (size_t i = 0; i < 100; i++)
	{
		_FLAG[i] = false;
	}
}

//中心点按从左往右排序
bool P_swap(Point a, Point b)
{
	return a.x < b.x;
}

//颜色与中心点定位
void color_target(vector<Point2f> center, vector<RotatedRect> m_ellipses, Mat srcImg)
{
	int area_temp;
	int Num_of_center = center.size();
	Rect RectTemp;
	vector<Rect> RectTarget;
	//目标区域框选
	for (size_t c = 0; c < Num_of_center; c++)
	{
		area_temp = 0;
		for (size_t i = 0; i < m_ellipses.size(); i++)
		{
			if (!color_flag[i])
			{
				if (abs(m_ellipses[i].center.x - center[c].x) < 10 && abs(m_ellipses[i].center.y - center[c].y) < 10)
				{
					color_flag[i] = true;
					if (area_temp < (m_ellipses[i].boundingRect()).area())
					{
						area_temp = (m_ellipses[i].boundingRect()).area();
						RectTemp = m_ellipses[i].boundingRect();
					}
				}
			}
			else continue;
		}
		RectTarget.push_back(RectTemp);
	}
	
	vector<int> H_Average;
	for (size_t c = 0; c < Num_of_center; c++)
	{
		Mat _COPY = srcImg(RectTarget[c]);
		Mat mask = _COPY.clone();
		//上面两步是调试用的，实际用的话直接这么写：
		//Mat mask = srcImg(RectTarget[c]);

		cvtColor(mask, mask, COLOR_BGR2HSV);
		//设置像素遍历迭代器
		MatConstIterator_<Vec3b> maskStart = mask.begin<Vec3b>();
		MatConstIterator_<Vec3b> maskEnd = mask.end<Vec3b>();
		int H_Val = 0;
		for (; maskStart != maskEnd; maskStart++)
		{
			H_Val += (*maskStart)[0];
		}
		H_Val /= mask.cols * mask.rows;
		H_Average.push_back(H_Val);//保存当前区域色相H的平均值
	}
	int color[4] = { -1, -1, -1, -1 };//B: 0, G: 1, R: 2
	//色相H的大小为：绿色 < 蓝色 < 红色，因此可以比较H平均值大小，从而分析出区域的颜色

	int index = 0;
	int index_BGR[] = {1, 0, 2};
	//那个关于找颜色与H的对应关系我用的办法比较笨，关于这方面的算法没怎么联系过，你看看有什么更好的方法
	for (size_t i = 25; i < 180; i++)
	{
		for (size_t c = 0; c < Num_of_center; c++)
		{
			if (H_Average[c] == i)
			{
				color[c] = index_BGR[index];
				index++;
			}
		}
	}
	//控制台输出，调试用
	for (size_t c = 0; c < 4; c++)
	{
		cout << color[c];
		if (c < Num_of_center)
		{
			cout << "  center.x: " << center[c].x << endl;
		}

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
		vector<RotatedRect> m_ellipses;//第一次初筛后椭圆容器
		vector<Point2f> center;//目标椭圆中心容器
		
		srcdst = srcImg.clone();
		srcCopy = srcdst.clone();

		GaussianBlur(srcdst, srcdst, Size(Gauss_size, Gauss_size), 0, 0);
		cvtColor(srcdst, srcdst, COLOR_BGR2GRAY);
		Canny(srcdst, srcdst, Canny_low, Canny_up, 3);
		//imshow("step1.", srcdst);//用于调试
		//ROI设置
		Mat mm = srcCopy(Rect(0, 0, srcCopy.cols, srcCopy.rows));
		mm = { Scalar(0, 0, 0) };//把ROI中的像素值改为黑色

		vector<vector<Point> > contours;// 创建容器，存储轮廓
		vector<Vec4i> hierarchy;// 寻找轮廓所需参数
		findContours(srcdst, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

		//如果查找到了轮廓
		if (contours.size())
		{
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
				RotatedRect m_ellipsetemp;//创建临时接收椭圆的容器
				m_ellipsetemp = fitEllipse(contours[i]);  //找到的第一个轮廓，放置到m_ellipsetemp
				if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.2)
					continue;
				_center = m_ellipsetemp.center;//读取椭圆中心，必要
				centers.push_back(_center);
				m_ellipses.push_back(m_ellipsetemp);

			}
		}

		//聚类
		flag_init(jvlei_flag);
		for (int i = 0; i < centers.size() - 2; i++)
		{
			jvlei_flag[i] = true;
			int x_temp = centers[i].x, y_temp = centers[i].y, count = 1;
			for (int j = 1; j < centers.size(); j++)
			{
				if (abs(centers[i].x - centers[j].x) < 10 && abs(centers[i].y - centers[j].y) < 10)
				{
					if (!jvlei_flag[j])
					{
						jvlei_flag[j] = true;
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
		cout << "个数： " << center.size() << endl;//用于调试
		//颜色标定
		flag_init(color_flag);
		sort(center.begin(), center.end(), P_swap);//按从左往右的顺序排序
		color_target(center, m_ellipses, srcImg);

		//绘制中心十字，用于调试
		for (size_t i = 0; i < center.size(); i++)
		{
			drawCross(srcImg, center[i], Scalar(0, 0, 255), 30, 2);
		}

		imshow("srcImg", srcImg);//用于调试
	}
}