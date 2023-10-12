#include<opencv.hpp>
#include<iostream>
#include"usings.hpp"//���ͷ�ļ������Լ�Ϊ������ñ��ͶȺ����������

cv::Scalar R_Low = cv::Scalar(150, 50, 50);
cv::Scalar R_up = cv::Scalar(180, 255, 255);

cv::Scalar G_Low = cv::Scalar(50, 50, 46);
cv::Scalar G_up = cv::Scalar(85, 255, 255);

cv::Scalar B_Low = cv::Scalar(80, 43, 43);
cv::Scalar B_up = cv::Scalar(130, 255, 255);

cv::Scalar O_Low = cv::Scalar(12, 100, 100);
cv::Scalar O_up = cv::Scalar(25, 255, 255);

cv::Scalar Low[3] = { B_Low, G_Low, R_Low };
cv::Scalar Up[3] = { B_up, G_up, R_up };

cv::Scalar B_color = cv::Scalar(255, 0, 0);
cv::Scalar G_color = cv::Scalar(0, 255, 0);
cv::Scalar R_color = cv::Scalar(0, 0, 255);
cv::Scalar DrawingColor[3] = { B_color ,G_color ,R_color };

cv::Mat color_mask(cv::Mat srcImg)
{
	cv::Mat Img = srcImg.clone();
	//���Ͷ�+20, ���ܼ�̫��
	Img = Saturation(Img, 20);
	
	unsigned RGB = 0;//�����ã�ͬʱ���3����ɫ

	cv::Mat HSVImg;
	cv::cvtColor(Img, HSVImg, cv::COLOR_BGR2HSV);

	for (; RGB < 3; RGB++)
	{
		cv::Mat img_m;
		cv::inRange(HSVImg, Low[RGB], Up[RGB], img_m);//��ȡ��ɫ
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
		morphologyEx(img_m, img_m, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);//����������ֻ��һ��
		std::vector<std::vector<cv::Point> >contours;//��������
		cv::Point2f vtx[4];//���ζ�������
		cv::findContours(img_m, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);//����������
		if (!contours.empty())
		{
			//ѡȡ������ɫ��
			int tag = 0;
			double area_of_contour = cv::contourArea(contours[0]);
			for (int i = 1; i < contours.size(); i++)
			{
				if (area_of_contour < cv::contourArea(contours[i]))
				{
					tag = i;
					area_of_contour = cv::contourArea(contours[i]);
				}
			}

			//��������
			cv::drawContours(Img, contours, tag, DrawingColor[RGB], 1);
			//���ƾ��Σ�������β�����ת���Σ�������ͨ����
			cv::Rect rect = cv::boundingRect(contours[tag]);
			cv::rectangle(Img, rect, DrawingColor[RGB], 2);
		}
	}
	return Img;
}