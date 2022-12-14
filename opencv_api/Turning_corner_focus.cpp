#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
Scalar BK_low_ = Scalar(0, 0, 0);
Scalar BK_up_ = Scalar(180, 255, 100);
const int judge_low = 50, judge_up = 200;
const int x_low = 190, x_up = 290, y_low = 50, y_up = 270;
RNG rng_(12345);
void Turning_Corner_focus(VideoCapture* camera)
{
	Mat srcF, srcImg;
	bool flag = false;//是否检测到了线
	while (true)
	{
		*camera >> srcImg;
		if (srcImg.empty())
			break;
		if ((char)waitKey(1) == 27)
		{
			break;
			//destroyAllWindows();//关闭所有图形窗口
		}
		resize(srcImg, srcF, srcImg.size() / 4);
		srcF = srcF(Range(y_low, y_up), Range(x_low, x_up));//动态调整
#if(1)//色彩分离查找车道线
		cvtColor(srcF, srcF, COLOR_BGR2HSV);
		inRange(srcF, BK_low_, BK_up_, srcF);
#else //灰度阈值查找车道线
		cvtColor(srcF, srcF, COLOR_BGR2GRAY);
		int thresh = 200;
		threshold(srcF,srcF,thresh, 255, THRESH_BINARY);
#endif
#if(0)//可能需要去除干扰轮廓
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(srcF, srcF, MORPH_OPEN, element);
#endif
		vector<vector<Point> >contours;//轮廓容器
		findContours(srcF, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//查找轮廓
		Mat dst = Mat::zeros(srcF.size(), CV_8UC3);//创建空白图像，调试时查看轮廓使用
		int cX = 0, cY = 0;
		int tag = 0;
		//如果检测到了车道线
		if (contours.size())//容器必须要非空
		{
			flag = true;
			for (size_t i = 0; i < contours.size(); i++)
			{
				Moments moment = moments(contours[i]);
				if (moment.m00)//除数不能为0
				{
					//求取轮廓重心的X坐标
					cX = cvRound(moment.m10 / moment.m00);
					//求取轮廓重心的Y坐标
					cY = cvRound(moment.m01 / moment.m00);
				}
				drawContours(dst, contours, i, Scalar(rng_.uniform(0, 255), rng_.uniform(0, 255), rng_.uniform(0, 255)), 1);//绘制轮廓
				line(dst, Point(cX, cY), Point(cX, cY), Scalar(0, 0, 255), 2, LINE_AA);//绘制中心点
				tag += cY;
			}
#if(1)//如果检测出了多个相互分离的轮廓，则求平均值
			tag = int(tag / contours.size());
#endif
			//tag信息写入图片中
			char info[40] = {};
			sprintf_s(info, "tag: %d\ntag_low: %d\ntag_up: %d", tag, judge_low, judge_up);
			putText(dst, info, Point(8, 20), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
			imshow("dst", dst);
			//cout << "Tag:  " << tag << endl;
			//在范围内执行转弯
			if (tag >= judge_low && tag <= judge_up)
				cout << "执行转弯" << endl;
			else if (tag > judge_up)
				cout << "立即停止，调整位置，转弯" << endl;
		}
		//如果未检测到车道线：1.之前从未检测到>>继续运行；2.之前检测到>>可能即将或已经压线，立即停止
		else
		{
			if (flag)
			{
				//flag = false;
				cout << "即将越界，立即停止" << endl;
			}
		}
	}
}