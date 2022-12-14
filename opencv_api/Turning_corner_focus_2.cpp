#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
Scalar BK_low_ = Scalar(0, 0, 0);
Scalar BK_up_ = Scalar(180, 255, 100);
const int judge_low = 50, judge_up = 200;
const int x_low = 190, x_up = 290, y_low = 50, y_up = 270;
RNG rng_(12345);
//ת���ж�����
void Turning_Corner_focus_judge(Mat diff)
{
	bool flag = false;//�Ƿ��⵽����
	vector<vector<Point> >contours;//��������
	findContours(diff, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//��������
	Mat dst = Mat::zeros(diff.size(), CV_8UC3);//�����հ�ͼ�񣬵���ʱ�鿴����ʹ��
	int cX = 0, cY = 0;
	int tag = 0;
	//�����⵽�˳�����
	if (contours.size())//��������Ҫ�ǿ�
	{
		flag = true;
		for (size_t i = 0; i < contours.size(); i++)
		{
			Moments moment = moments(contours[i]);
			if (moment.m00)//��������Ϊ0
			{
				//��ȡ�������ĵ�X����
				cX = cvRound(moment.m10 / moment.m00);
				//��ȡ�������ĵ�Y����
				cY = cvRound(moment.m01 / moment.m00);
			}
			drawContours(dst, contours, i, Scalar(rng_.uniform(0, 255), rng_.uniform(0, 255), rng_.uniform(0, 255)), 1);//��������
			line(dst, Point(cX, cY), Point(cX, cY), Scalar(0, 0, 255), 2, LINE_AA);//�������ĵ�
			tag += cY;
		}
#if(1)//��������˶���໥���������������ƽ��ֵ
		tag = int(tag / contours.size());
#endif
		//tag��Ϣд��ͼƬ��
		//char info[40] = {};
		//sprintf_s(info, "tag: %d\ntag_low: %d\ntag_up: %d", tag, judge_low, judge_up);
		//putText(dst, info, Point(8, 20), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
		imshow("dst", dst);
		//cout << "Tag:  " << tag << endl;
		//�ڷ�Χ��ִ��ת��
		if (tag >= judge_low && tag <= judge_up)
			cout << "ִ��ת��" << endl;
		else if (tag > judge_up)
			cout << "����ֹͣ������λ�ã�ת��" << endl;
	}
	//���δ��⵽�����ߣ�1.֮ǰ��δ��⵽>>�������У�2.֮ǰ��⵽>>���ܼ������Ѿ�ѹ�ߣ�����ֹͣ
	else
	{
		if (flag)
		{
			//flag = false;
			cout << "����Խ�磬����ֹͣ" << endl;
		}
	}
}
//֡��ж��߽���
void moveCheck(Mat& frontImg, Mat& afterImg)
{
	Mat frontGray, afterGray, diff;
	cvtColor(frontImg, frontGray, COLOR_BGR2GRAY);
	cvtColor(afterImg, afterGray, COLOR_BGR2GRAY);
	//֡��� �ҵ�֡��֮֡���˶��������
	absdiff(frontGray, afterGray, diff);
	//��ֵ��
	int thresh = 150;//��ֵ��Ҫ��̬����
	threshold(diff, diff, thresh, 255, THRESH_BINARY);
	//imshow("threashold",diff);
	Mat element = cv::getStructuringElement(MORPH_RECT, Size(3, 3));//��ʴ����Ҫ��̬����
	erode(diff, diff, element);
	//���ʹ���
	Mat element2 = cv::getStructuringElement(MORPH_RECT, Size(10, 10)); //Ҫ��̬����
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
		frame = frame(Range(y_low, y_up), Range(x_low, x_up));//��̬����
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