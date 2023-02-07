#include <opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
//和之前camshift一样，我为了调试方便，先用颜色识别找到目标物体
Scalar R_Low = Scalar(156, 100, 80);
Scalar R_up = Scalar(180, 255, 255);

Scalar G_Low = Scalar(30, 50, 46);
Scalar G_up = Scalar(85, 255, 255);

Scalar B_Low = Scalar(85, 100, 100);
Scalar B_up = Scalar(120, 255, 255);

Scalar Low[3] = { B_Low, G_Low, R_Low };
Scalar Up[3] = { B_up, G_up, R_up };

int scale = 1;//图片缩放倍数，调试用

//寻找目标
bool TargetFinding(Mat srcImg, Rect* rect_, unsigned COLOR)
{
	Mat srcF;
	GaussianBlur(srcImg, srcF, Size(3, 3), 0, 0);
	cvtColor(srcF, srcF, COLOR_BGR2HSV);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
	inRange(srcF, Low[COLOR], Up[COLOR], srcF);
	erode(srcF, srcF, element);
	vector<vector<Point> >contours;
	findContours(srcF, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	if (!contours.empty())
	{
		*rect_ = boundingRect(contours[0]);
		float Rect_area = (*rect_).area();
		int tag = 0;
		for (int i = 1; i < contours.size(); i++)
		{
			Rect rect_temp = boundingRect(contours[i]);
			float Rect_area_temp = rect_temp.area();
			if (Rect_area_temp > Rect_area)
			{
				tag = i;
				*rect_ = rect_temp;
				Rect_area = Rect_area_temp;
			}
		}
		return true;
	}
	else return false;
}

vector<Point2f> pts;//存放预测点的容器，用于绘制预测的轨迹，调试用

//得到目标的中心点坐标，这个是和TargetFinding函数结合使用
Point getCenterPoint(Rect rect)
{
	Point cpt;
	cpt.x = rect.x + cvRound(rect.width / 2.0);
	cpt.y = rect.y + cvRound(rect.height / 2.0);
	return cpt;
}

//绘制预测的中心点十字，调试用
void drawCross(Mat& img, Point2f point, Scalar color, int size, int thickness = 1)
{
	line(img, Point(point.x - size / 2, point.y), Point(point.x + size / 2, point.y), color, thickness, 8, 0);
	line(img, Point(point.x, point.y - size / 2), Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

int main()
{
	//读取视频
	VideoCapture video;
	video.open("E:\\opencv_test\\car\\VID2.mp4");
	if (!video.isOpened())
		return -1;

//核心代码：卡尔曼滤波初始化
	const int stateNum = 4;                                                                      //状态值4×1向量(x, y, △x, △y)
	const int measureNum = 2;                                                                    //测量值2×1向量(x, y)
	KalmanFilter KF(stateNum, measureNum, 0);                                                    //初始化卡尔曼滤波器
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1); //转移矩阵A
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定

	namedWindow("kalman");                                                         //创建窗口，用于查看结果
	
	Mat frame;     //视频帧

	//先找到目标物体，这一步是为了配合卡尔曼滤波，要给卡尔曼滤波一个初始值
	Rect rect;     //目标物体矩形
	bool targetFound = false;
	while (!targetFound)
	{
		video >> frame;
		resize(frame, frame, frame.size() / scale);//重设大小
		targetFound = TargetFinding(frame, &rect, 0);
		if (targetFound)
			imshow("rect", frame(rect));
	}

//核心代码：初始状态值x(0)，相当于是拿到了第一个原始数据
	KF.statePost = (Mat_<float>(4, 1) << getCenterPoint(rect).x, getCenterPoint(rect).y, 0, 0);  

	while (true)
	{
		video >> frame;
		if (frame.empty())
			break;
		if ((char)waitKey(1) == 27)
			break;
		resize(frame, frame, frame.size() / scale);
//核心代码：卡尔曼预测
		Mat prediction = KF.predict();
		Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x', y')
		pts.push_back(predict_pt);

//核心代码：进行下一次预测更新
		TargetFinding(frame, &rect, 0);
		measurement.at<float>(0) = (float)getCenterPoint(rect).x;
		measurement.at<float>(1) = (float)getCenterPoint(rect).y;
		KF.correct(measurement);
//绘制
		drawCross(frame, predict_pt, Scalar(255, 255, 0), 30, 3);
		for (int i = 0; i < pts.size() - 1; i++)
		{
			line(frame, pts[i], pts[i + 1], Scalar(0, 255, 0), 2);
		}
		imshow("kalman", frame);
	}

	waitKey(0);
	return 0;
}