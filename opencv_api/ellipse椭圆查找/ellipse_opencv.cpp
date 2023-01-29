#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;

const int Gauss_size = 3;//��˹ƽ���ں˴�С
const int Canny_low = 50;//��һ��Canny��Ե���ҵĵ�һ�ͺ�����
const int Canny_up = 100;//��һ��Canny��Ե���ҵĵڶ��ͺ�����

const int con_Area_min = 4500;//��ɸ-��С�����ֵ
const int con_Point_cont = 20;//��ɸ-ͼ�����ٵ������ֵ��������ĳ����������ĵ�ĸ��������ڸ���ֵ����������Ч
const int con_Area_max = 200000;//��ɸ-��������ֵ

//ֱ��б�ʴ������ԭ�����ز���
const int line_Point_1 = 10;//��1�������ֱ�ʾΧ�ɷ�����������ţ�ֻҪ��̫���׼���
const int line_Point_2 = 20;//��2
const int line_threshold = 0.5;//�ж���ֵ��С�ڼ��ж�Ϊֱ��

bool flag[100];//�����ʶλ

//��ԲԲ��ʮ�ֹ����ƣ����ڵ��Թ۲�
void drawCross(Mat& img, Point2f point, Scalar color, int size, int thickness = 1)
{
	//���ƺ���
	line(img, Point(point.x - size / 2, point.y), Point(point.x + size / 2, point.y), color, thickness, 8, 0);
	//��������
	line(img, Point(point.x, point.y - size / 2), Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

//��ʶλ��ʼ��
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
		
		Mat srcdst, srcCopy;//�������������Ҫ����ͼƬ
		Point2f _center;//��Բ����
		vector<Point2f> centers;//��Բ��������
		vector<Point2f> center;//Ŀ����Բ����
		resize(srcImg, srcdst, srcImg.size());//�����С����ѡ
		srcCopy = srcdst.clone();

		//��һ��Ԥ����
		GaussianBlur(srcdst, srcdst, Size(Gauss_size, Gauss_size), 0, 0);
		cvtColor(srcdst, srcdst, COLOR_BGR2GRAY);
		Canny(srcdst, srcdst, Canny_low, Canny_up, 3);
		//imshow("step1.", srcdst);//���ڵ���
		//ROI����
		Mat mm = srcCopy(Rect(0, 0, srcCopy.cols, srcCopy.rows));
		mm = { Scalar(0, 0, 0) };//��ROI�е�����ֵ��Ϊ��ɫ

		//��һ����������
		vector<vector<Point> > contours;// �����������洢����
		vector<Vec4i> hierarchy;// Ѱ�������������
		findContours(srcdst, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

		//Mat imageContours = Mat::zeros(mm.size(), CV_8UC1);//��������չʾͼ�����ڵ���
		//������ҵ�������
		if (contours.size())
		{
			
			//����չʾ�����ڵ���
			/*
			for (int i = 0; i < contours.size(); i++)
			{
				drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
			}
			imshow("Contours_1", imageContours);
			*/
			//��һ���ų�
			for (int i = 0; i < contours.size(); i++)
			{
				//��ɸ
				if (contourArea(contours[i]) < con_Area_min || contours[i].size() < con_Point_cont || contourArea(contours[i]) > con_Area_max)
					continue;
				//����ֱ��б�ʴ�����ȵ�ԭ��
				if (abs(((double)(contours[i][0].y - contours[i][line_Point_1].y) / (double)(contours[i][0].x - contours[i][line_Point_1].x) - (double)(contours[i][line_Point_1].y - contours[i][line_Point_2].y) / (double)(contours[i][line_Point_1].x - contours[i][line_Point_2].x))) < line_threshold)
					continue;
				//���ð�͹�Ե�ԭ��
				if (!abs((contours[i][0].y + contours[i][20].y) / 2 - contours[i][10].y))
					continue;
				//drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
				RotatedRect m_ellipsetemp;//����������Բ������
				m_ellipsetemp = fitEllipse(contours[i]);  //�ҵ��ĵ�һ�����������õ�m_ellipsetemp
				if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.2)
					continue;
				ellipse(mm, m_ellipsetemp, cv::Scalar(255, 255, 255));//��ͼ���л�����Բ����Ҫ
			}
			//imshow("Contours_1", imageContours);
			imshow("mm", mm);//��ʾ��һ���ų���������ڵ���
		}
		//�ڶ���Ԥ����
		Mat mmdst;
		GaussianBlur(mm, mmdst, Size(Gauss_size, Gauss_size), 0, 0);
		cvtColor(mmdst, mmdst, COLOR_BGR2GRAY);
		Canny(mmdst, mmdst, 50, 150, 3);
		//imshow("step2.", mmdst);//���ڵ���
		//�ڶ�����������
		vector<vector<Point> > contours1;// �����������洢����
		vector<Vec4i> hierarchy1;// Ѱ�������������
		findContours(mmdst, contours1, hierarchy1, RETR_CCOMP, CHAIN_APPROX_NONE);
		//Mat imageContours1 = Mat::zeros(mmdst.size(), CV_8UC1);//��������չʾͼ�����ڵ���
		//������ҵ�������
		if (contours1.size())
		{
			//�ڶ���ɸ��
			for (int i = 0; i < contours1.size(); i++)
			{
				//��ɸ
				if (contours1[i].size() < 10 || contours1[i].size() > 1000)
					continue;
				//���ð�͹�Ե�ԭ��
				//if (!abs((contours1[i][0].y + contours1[i][20].y) / 2 - contours1[i][10].y))
				//	continue;
				if (contourArea(contours1[i]) < 4500 || contourArea(contours1[i]) > 180000)
					continue;
				//drawContours(imageContours1, contours1, i, Scalar(255), 1, 8);//���ڵ���

				RotatedRect m_ellipsetemp1;
				m_ellipsetemp1 = fitEllipse(contours1[i]);
				ellipse(srcImg, m_ellipsetemp1, cv::Scalar(255, 0, 0));//������Բ�����ڵ���
				_center = m_ellipsetemp1.center;//��ȡ��Բ���ģ���Ҫ
				//drawCross(srcCopy, _center, Scalar(255, 0, 0), 30, 2);//��������ʮ�֣����ڵ���
				circle(srcImg, _center, 1, Scalar(0, 255, 0), -1);  // ���뾶Ϊ1��Բ(���㣩, ���ڵ���
				centers.push_back(_center);
			}
			//imshow("Contours_2", imageContours1);//���ڵ���
			//cout << centers.size() << endl;
			//����
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
				//ƽ������������ģ��о���̫�׵������Ǿ��ȸо����У�׷�󾫶ȵĻ������� Weiszfeld �㷨����λ���ģ��Ǹ�Ҫ����
				center.push_back(Point((int)x_temp / count, (int)y_temp / count));
				}
			}
			cout << "��Բ������ " << center.size() << endl;
			//��������ʮ�֣����ڵ���
			for (size_t i = 0; i < center.size(); i++)
			{
				drawCross(srcImg, center[i], Scalar(0, 0, 255), 30, 2);
			}
			//��ɫ�궨
			//Ŀǰ�Ͳ���һ����
		}
		imshow("srcCopy", srcImg);//���ڵ���
	}
}