#include<opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//��ɫ��ǿ������غ궨��
#define max2(a,b) (a>b?a:b)
#define max3(a,b,c) (a>b?max2(a,c):max2(b,c))
#define min2(a,b) (a<b?a:b)
#define min3(a,b,c) (a<b?min2(a,c):min2(b,c))

cv::Mat Saturation(cv::Mat src, int percent);//��ɫ��ǿ����

cv::Mat LBD_Thershold_Func(cv::Mat Enhanced);//����Ӧ��ֵ�����ҳ����߽�
cv::Mat LBD_Color_Func(cv::Mat Enhanced);//��ɫinrange���ҳ����߽�
//��ֹ7.3��д��������

void LBD_(cv::Mat Enhanced)//�߽���ҷ������ú���
{
	GaussianBlur(Enhanced, Enhanced, Size(3, 3), 0, 0);//�˲�Ԥ����

	//����һ
	//Mat LineImg = LBD_Thershold_Func(Enhanced);
	//imshow("LineImg", LineImg);

	//������
	Mat ColorImg = LBD_Color_Func(Enhanced);
	imshow("LineImg", ColorImg);

}

//��ɫ��ǿ�ĳ����߽����----������
void LBD_ColorEnhanced(Mat Img)
{
	resize(Img, Img, Size(Img.cols / 4, Img.rows / 4));//����ͼ��
	imshow("original", Img);

	cv::Mat Enhanced = Saturation(Img, 100);//���Ͷȵ���������-100 �� 100, �������Ͷ���ǿ���������Ͷȼ���
	imshow("Enhanced", Enhanced);

	LBD_(Enhanced);//���ұ߽�
}

//��ɫ��ǿ����
cv::Mat Saturation(cv::Mat src, int percent)
{
	float Increment = percent * 1.0f / 100;
	cv::Mat temp = src.clone();
	int row = src.rows;
	int col = src.cols;
	for (int i = 0; i < row; ++i)
	{
		uchar* t = temp.ptr<uchar>(i);
		uchar* s = src.ptr<uchar>(i);
		for (int j = 0; j < col; ++j)
		{
			uchar b = s[3 * j];
			uchar g = s[3 * j + 1];
			uchar r = s[3 * j + 2];
			float max = max3(r, g, b);
			float min = min3(r, g, b);
			float delta, value;
			float L, S, alpha;
			delta = (max - min) / 255;
			if (delta == 0)
				continue;
			value = (max + min) / 255;
			L = value / 2;
			if (L < 0.5)
				S = delta / value;
			else
				S = delta / (2 - value);
			if (Increment >= 0)
			{
				if ((Increment + S) >= 1)
					alpha = S;
				else
					alpha = 1 - Increment;
				alpha = 1 / alpha - 1;
				t[3 * j + 2] = static_cast<uchar>(r + (r - L * 255) * alpha);
				t[3 * j + 1] = static_cast<uchar>(g + (g - L * 255) * alpha);
				t[3 * j] = static_cast<uchar>(b + (b - L * 255) * alpha);
			}
			else
			{
				alpha = Increment;
				t[3 * j + 2] = static_cast<uchar>(L * 255 + (r - L * 255) * (1 + alpha));
				t[3 * j + 1] = static_cast<uchar>(L * 255 + (g - L * 255) * (1 + alpha));
				t[3 * j] = static_cast<uchar>(L * 255 + (b - L * 255) * (1 + alpha));
			}
		}
	}
	return temp;
}

//����Ӧ��ֵ�����ҳ����߽�
cv::Mat LBD_Thershold_Func(cv::Mat Enhanced)
{
	Mat gray;
	cvtColor(Enhanced, gray, COLOR_BGR2GRAY);
	imshow("Gray", gray);
	Mat ThreImg;
	adaptiveThreshold(gray, ThreImg, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, -1);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	morphologyEx(ThreImg, ThreImg, MORPH_CLOSE, element, Point(-1, -1), 3);
	imshow("Thresh", ThreImg);

	Canny(ThreImg, ThreImg, 50, 200, 3);
	imshow("edge", ThreImg);

	Mat LineImg;
	cvtColor(ThreImg, LineImg, COLOR_GRAY2BGR);
	vector<Vec2f> lines;
	HoughLines(ThreImg, lines, 1, CV_PI / 180, 100, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(LineImg, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
	}
	return LineImg;
}

//��ɫinrange���ҳ����߽�
cv::Mat LBD_Color_Func(cv::Mat Enhanced)
{
	Mat HSVImg;
	cvtColor(Enhanced, HSVImg, COLOR_BGR2HSV);
	imshow("HSVImg", HSVImg);

	Scalar Boundary_low = Scalar(78, 0, 0);
	Scalar Boundary_high = Scalar(179, 255, 255);
	Mat RangeImg;
	inRange(HSVImg, Boundary_low, Boundary_high, RangeImg);
	imshow("RangeImg", RangeImg);

	Mat edgeImg;
	Canny(RangeImg, edgeImg, 50, 200, 3);
	imshow("edge", edgeImg);

	Mat LineImg;
	cvtColor(edgeImg, LineImg, COLOR_GRAY2BGR);
	vector<Vec2f> lines;
	HoughLines(edgeImg, lines, 1, CV_PI / 180, 100, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(LineImg, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
	}
	return LineImg;

}