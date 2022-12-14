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
	bool flag = false;//�Ƿ��⵽����
	while (true)
	{
		*camera >> srcImg;
		if (srcImg.empty())
			break;
		if ((char)waitKey(1) == 27)
		{
			break;
			//destroyAllWindows();//�ر�����ͼ�δ���
		}
		resize(srcImg, srcF, srcImg.size() / 4);
		srcF = srcF(Range(y_low, y_up), Range(x_low, x_up));//��̬����
#if(1)//ɫ�ʷ�����ҳ�����
		cvtColor(srcF, srcF, COLOR_BGR2HSV);
		inRange(srcF, BK_low_, BK_up_, srcF);
#else //�Ҷ���ֵ���ҳ�����
		cvtColor(srcF, srcF, COLOR_BGR2GRAY);
		int thresh = 200;
		threshold(srcF,srcF,thresh, 255, THRESH_BINARY);
#endif
#if(0)//������Ҫȥ����������
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(srcF, srcF, MORPH_OPEN, element);
#endif
		vector<vector<Point> >contours;//��������
		findContours(srcF, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//��������
		Mat dst = Mat::zeros(srcF.size(), CV_8UC3);//�����հ�ͼ�񣬵���ʱ�鿴����ʹ��
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
			char info[40] = {};
			sprintf_s(info, "tag: %d\ntag_low: %d\ntag_up: %d", tag, judge_low, judge_up);
			putText(dst, info, Point(8, 20), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
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
}