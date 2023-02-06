#include <opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
//��֮ǰcamshiftһ������Ϊ�˵��Է��㣬������ɫʶ���ҵ�Ŀ������
Scalar R_Low = Scalar(156, 100, 80);
Scalar R_up = Scalar(180, 255, 255);

Scalar G_Low = Scalar(30, 50, 46);
Scalar G_up = Scalar(85, 255, 255);

Scalar B_Low = Scalar(85, 100, 100);
Scalar B_up = Scalar(120, 255, 255);

Scalar Low[3] = { B_Low, G_Low, R_Low };
Scalar Up[3] = { B_up, G_up, R_up };

int scale = 1;//ͼƬ���ű�����������

//Ѱ��Ŀ��
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

vector<Point2f> pts;//���Ԥ�������������ڻ���Ԥ��Ĺ켣��������

//�õ�Ŀ������ĵ����꣬����Ǻ�TargetFinding�������ʹ��
Point getCenterPoint(Rect rect)
{
	Point cpt;
	cpt.x = rect.x + cvRound(rect.width / 2.0);
	cpt.y = rect.y + cvRound(rect.height / 2.0);
	return cpt;
}

//����Ԥ������ĵ�ʮ�֣�������
void drawCross(Mat& img, Point2f point, Scalar color, int size, int thickness = 1)
{
	line(img, Point(point.x - size / 2, point.y), Point(point.x + size / 2, point.y), color, thickness, 8, 0);
	line(img, Point(point.x, point.y - size / 2), Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

int main()
{
	//��ȡ��Ƶ
	VideoCapture video;
	video.open("E:\\opencv_test\\car\\VID2.mp4");
	if (!video.isOpened())
		return -1;

//���Ĵ��룺�������˲���ʼ��
	const int stateNum = 4;                                                                      //״ֵ̬4��1����(x, y, ��x, ��y)
	const int measureNum = 2;                                                                    //����ֵ2��1����(x, y)
	KalmanFilter KF(stateNum, measureNum, 0);                                                    //��ʼ���������˲���
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1); //ת�ƾ���A
	setIdentity(KF.measurementMatrix);                                             //��������H
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //ϵͳ�����������Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //���������������R
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //����������Э�������P
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //��ʼ����ֵx'(0)����Ϊ����Ҫ�������ֵ�����Ա����ȶ�

	namedWindow("kalman");                                                         //�������ڣ����ڲ鿴���
	
	Mat frame;     //��Ƶ֡

	//���ҵ�Ŀ�����壬��һ����Ϊ����Ͽ������˲���Ҫ���������˲�һ����ʼֵ
	Rect rect;     //Ŀ���������
	bool targetFound = false;
	while (!targetFound)
	{
		video >> frame;
		resize(frame, frame, frame.size() / scale);//�����С
		targetFound = TargetFinding(frame, &rect, 0);
		if (targetFound)
			imshow("rect", frame(rect));
	}

//���Ĵ��룺��ʼ״ֵ̬x(0)���൱�����õ��˵�һ��ԭʼ����
	KF.statePost = (Mat_<float>(4, 1) << getCenterPoint(rect).x, getCenterPoint(rect).y, 0, 0);  

	while (true)
	{
		video >> frame;
		if (frame.empty())
			break;
		if ((char)waitKey(1) == 27)
			break;
		resize(frame, frame, frame.size() / scale);
//���Ĵ��룺������Ԥ��
		Mat prediction = KF.predict();
		Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //Ԥ��ֵ(x', y')
		pts.push_back(predict_pt);

//���Ĵ��룺������һ��Ԥ�����
		TargetFinding(frame, &rect, 0);
		measurement.at<float>(0) = (float)getCenterPoint(rect).x;
		measurement.at<float>(1) = (float)getCenterPoint(rect).y;
		KF.correct(measurement);
//����
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