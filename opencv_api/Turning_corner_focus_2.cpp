#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
Scalar BK_low_ = Scalar(0, 0, 0);
Scalar BK_up_ = Scalar(180, 255, 100);
const int judge_low = 50, judge_up = 200;
const int x_low = 190, x_up = 290, y_low = 50, y_up = 270;
RNG rng_(12345);
//转弯判定函数
void Turning_Corner_focus_judge(Mat diff)
{
	bool flag = false;//是否检测到了线
	vector<vector<Point> >contours;//轮廓容器
	findContours(diff, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//查找轮廓
	Mat dst = Mat::zeros(diff.size(), CV_8UC3);//创建空白图像，调试时查看轮廓使用
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
		//char info[40] = {};
		//sprintf_s(info, "tag: %d\ntag_low: %d\ntag_up: %d", tag, judge_low, judge_up);
		//putText(dst, info, Point(8, 20), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
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
//帧差法判定边界线
void moveCheck(Mat& frontImg, Mat& afterImg)
{
	Mat frontGray, afterGray, diff;
	cvtColor(frontImg, frontGray, COLOR_BGR2GRAY);
	cvtColor(afterImg, afterGray, COLOR_BGR2GRAY);
	//帧差处理 找到帧与帧之间运动物体差异
	absdiff(frontGray, afterGray, diff);
	//二值化
	int thresh = 150;//阈值，要动态调整
	threshold(diff, diff, thresh, 255, THRESH_BINARY);
	//imshow("threashold",diff);
	Mat element = cv::getStructuringElement(MORPH_RECT, Size(3, 3));//腐蚀处理，要动态调整
	erode(diff, diff, element);
	//膨胀处理
	Mat element2 = cv::getStructuringElement(MORPH_RECT, Size(10, 10)); //要动态调整
	dilate(diff, diff, element2);
	//imshow("dilate",diff);
	Turning_Corner_focus_judge(diff);
}
void Turning_Corner_focus(VideoCapture* camera)
{
	Mat frame, temp;
	bool flag = false;
	while (camera->read(frame))
	{
		if ((char)waitKey(1) == 27)
		{
			break;
		}
		resize(frame, frame, frame.size() / 4);
		frame = frame(Range(y_low, y_up), Range(x_low, x_up));//动态调整
		if (!flag)
		{
			moveCheck(frame, frame);
			flag = true;
		}
		else
		{
			moveCheck(temp, frame);
		}
		temp = frame.clone();
	}
}