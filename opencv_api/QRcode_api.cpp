#include <opencv.hpp>
#include <iostream>
#include <fstream>
#include "QRcode.h"

//�������꣬�ٶ���ʾ���ֵ�ͼ��Ϊ1080p����Ԥ���
int ptx_info[6] = { 350,800,1240,350,800,1240 };
int pty_info[6] = { 430,430,430,950,950,950 };
//��������ƫ������ǰ�ڵ�����
int txt_Xoffset = 0;
int txt_Yoffset = 0;
//���ִ�С
int txt_size = 30;
//���ֺ��
int txt_thick = 50;

void QRcode(cv::VideoCapture *camera, std::string* info)
{
	cv::Mat QR;
	//������ʾ���ֵ�ͼƬ���ٶ�1080p��3ͨ������ɫ����
	cv::Mat resImg(cv::Size(1920, 1080), CV_8UC3, cv::Scalar(100, 80, 60));
	std::cout << "\n\n��ʼʶ���ά��...\n\n" << std::endl;
	while (true)
	{
		*camera >> QR;
		if (!QR.data)
		{
			std::cout << "No data!" << std::endl;
			break;
		}
		cvtColor(QR, QR, cv::COLOR_BGR2GRAY);
		//�������
		cv::Mat straight_qrcode;
		cv::QRCodeDetector QR_detector;
		std::vector<cv::Point> points;
		*info = QR_detector.detectAndDecode(QR, points, straight_qrcode);
		//���Ʋ���
		if ((*info).size())
		{
			int ii = 0;
			for (size_t i = 0; i < (*info).size(); i++)
			{
				if (i != 3)//����ʾ +
				{
					char txt_temp[2] = { (*info)[i] };
					putText(resImg, txt_temp, cv::Point(ptx_info[ii] + txt_Xoffset, pty_info[ii] + txt_Yoffset), cv::FONT_HERSHEY_PLAIN, txt_size, cv::Scalar::all(255), txt_thick, cv::FILLED, false);
					ii++;
				}
			}
			std::cout << "\n\nʶ��ɹ�\n\n" << std::endl;
			imshow("resImg", resImg);
			break;
		}
	}
	
}
