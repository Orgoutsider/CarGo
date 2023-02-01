#include<opencv.hpp>
#include<iostream>
#include<stdlib.h>
using namespace cv;
using namespace std;

const int Gauss_size = 3;//��˹ƽ���ں˴�С
const int Canny_low = 50;//��һ��Canny��Ե���ҵĵ�һ�ͺ�����
const int Canny_up = 100;//��һ��Canny��Ե���ҵĵڶ��ͺ�����

const int con_Area_min = 3000;//��ɸ-��С�����ֵ
const int con_Point_cont = 15;//��ɸ-ͼ�����ٵ������ֵ��������ĳ����������ĵ�ĸ��������ڸ���ֵ����������Ч
const int con_Area_max = 150000;//��ɸ-��������ֵ

//ֱ��б�ʴ������ԭ�����ز���
const int line_Point_1 = 10;//��1�������ֱ�ʾΧ�ɷ�����������ţ�ֻҪ��̫���׼���
const int line_Point_2 = 20;//��2
const int line_threshold = 0.5;//�ж���ֵ��С�ڼ��ж�Ϊֱ��

bool jvlei_flag[100];//�����ʶλ
bool color_flag[100];//��ɫ��Ǳ�ʶλ

//��ԲԲ��ʮ�ֹ����ƣ����ڵ��Թ۲�
void drawCross(Mat& img, Point2f point, Scalar color, int size, int thickness = 1)
{
	//���ƺ���
	line(img, Point(point.x - size / 2, point.y), Point(point.x + size / 2, point.y), color, thickness, 8, 0);
	//��������
	line(img, Point(point.x, point.y - size / 2), Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

//��ʶλ��ʼ��
void flag_init(bool* _FLAG)
{
	for (size_t i = 0; i < 100; i++)
	{
		_FLAG[i] = false;
	}
}

//���ĵ㰴������������
bool P_swap(Point a, Point b)
{
	return a.x < b.x;
}

//��ɫ�����ĵ㶨λ
void color_target(vector<Point2f> center, vector<RotatedRect> m_ellipses, Mat srcImg)
{
	int area_temp;
	int Num_of_center = center.size();
	Rect RectTemp;
	vector<Rect> RectTarget;
	//Ŀ�������ѡ
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
		//���������ǵ����õģ�ʵ���õĻ�ֱ����ôд��
		//Mat mask = srcImg(RectTarget[c]);

		cvtColor(mask, mask, COLOR_BGR2HSV);
		//�������ر���������
		MatConstIterator_<Vec3b> maskStart = mask.begin<Vec3b>();
		MatConstIterator_<Vec3b> maskEnd = mask.end<Vec3b>();
		int H_Val = 0;
		for (; maskStart != maskEnd; maskStart++)
		{
			H_Val += (*maskStart)[0];
		}
		H_Val /= mask.cols * mask.rows;
		H_Average.push_back(H_Val);//���浱ǰ����ɫ��H��ƽ��ֵ
	}
	int color[4] = { -1, -1, -1, -1 };//B: 0, G: 1, R: 2
	//ɫ��H�Ĵ�СΪ����ɫ < ��ɫ < ��ɫ����˿��ԱȽ�Hƽ��ֵ��С���Ӷ��������������ɫ

	int index = 0;
	int index_BGR[] = {1, 0, 2};
	//�Ǹ���������ɫ��H�Ķ�Ӧ��ϵ���õİ취�Ƚϱ��������ⷽ����㷨û��ô��ϵ�����㿴����ʲô���õķ���
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
	//����̨�����������
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

		Mat srcdst, srcCopy;//�������������Ҫ����ͼƬ
		Point2f _center;//��Բ����
		vector<Point2f> centers;//��Բ��������
		vector<RotatedRect> m_ellipses;//��һ�γ�ɸ����Բ����
		vector<Point2f> center;//Ŀ����Բ��������
		
		srcdst = srcImg.clone();
		srcCopy = srcdst.clone();

		GaussianBlur(srcdst, srcdst, Size(Gauss_size, Gauss_size), 0, 0);
		cvtColor(srcdst, srcdst, COLOR_BGR2GRAY);
		Canny(srcdst, srcdst, Canny_low, Canny_up, 3);
		//imshow("step1.", srcdst);//���ڵ���
		//ROI����
		Mat mm = srcCopy(Rect(0, 0, srcCopy.cols, srcCopy.rows));
		mm = { Scalar(0, 0, 0) };//��ROI�е�����ֵ��Ϊ��ɫ

		vector<vector<Point> > contours;// �����������洢����
		vector<Vec4i> hierarchy;// Ѱ�������������
		findContours(srcdst, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

		//������ҵ�������
		if (contours.size())
		{
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
				RotatedRect m_ellipsetemp;//������ʱ������Բ������
				m_ellipsetemp = fitEllipse(contours[i]);  //�ҵ��ĵ�һ�����������õ�m_ellipsetemp
				if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.2)
					continue;
				_center = m_ellipsetemp.center;//��ȡ��Բ���ģ���Ҫ
				centers.push_back(_center);
				m_ellipses.push_back(m_ellipsetemp);

			}
		}

		//����
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
				//ƽ������������ģ��о���̫�׵������Ǿ��ȸо����У�׷�󾫶ȵĻ������� Weiszfeld �㷨����λ���ģ��Ǹ�Ҫ����
				center.push_back(Point((int)x_temp / count, (int)y_temp / count));
			}
		}
		cout << "������ " << center.size() << endl;//���ڵ���
		//��ɫ�궨
		flag_init(color_flag);
		sort(center.begin(), center.end(), P_swap);//���������ҵ�˳������
		color_target(center, m_ellipses, srcImg);

		//��������ʮ�֣����ڵ���
		for (size_t i = 0; i < center.size(); i++)
		{
			drawCross(srcImg, center[i], Scalar(0, 0, 255), 30, 2);
		}

		imshow("srcImg", srcImg);//���ڵ���
	}
}