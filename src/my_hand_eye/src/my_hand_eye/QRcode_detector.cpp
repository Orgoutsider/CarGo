#include "my_hand_eye/QRcode_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(my_hand_eye::QRcodeDetector, nodelet::Nodelet);

namespace my_hand_eye
{
	void QRcodeDetector::onInit()
	{
		nh_ = getNodeHandle();
		pnh_ = getPrivateNodeHandle();

		QR_code_publisher_ =
			nh_.advertise<my_hand_eye::ArrayofTaskArrays>("/task", 10,
														  boost::bind(&QRcodeDetector::connectCallback, this),
														  boost::bind(&QRcodeDetector::disconnectCallback, this));
		flag_ = false;
	}

	void QRcodeDetector::Callback(const std_msgs::StringConstPtr &info)
	{
		if (flag_)
		{
			ros::Duration(0.1).sleep();
			return;
		}
		cv::Mat resImg(cv::Size(1920, 1080), CV_8UC3, cv::Scalar(100, 80, 60));
		int ii = 0;
		my_hand_eye::ArrayofTaskArrays arr;
		arr.loop.reserve(2);
		std::string str = info->data;
		char num[6];
		for (size_t i = 0; i < str.size(); i++)
		{
			if (i != 3) // 不显示 +
			{
				char txt_temp[2] = {str[i]};
				putText(resImg, txt_temp, cv::Point(ptx_info[ii] + txt_Xoffset, pty_info[ii] + txt_Yoffset),
						cv::FONT_HERSHEY_PLAIN, txt_size, cv::Scalar::all(255), txt_thick, cv::FILLED, false);
				num[ii++] = str[i];
			}
		}
		ii = 0;
		for (int i = 0; i < 2; i++)
		{
			my_hand_eye::TaskArray tarr;
			uint8_t task;
			for (int j = 0; j < 3; j++)
			{
				task = num[ii++] - '0';
				tarr.task[i] = task;
			}
			arr.loop.push_back(tarr);
		}
		// NODELET_INFO("Succeeded to detect QR Code!");
		QR_code_publisher_.publish(arr);
		cv::namedWindow("resImg", cv::WINDOW_NORMAL);
		cv::setWindowProperty("resImg", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		cv::resize(resImg, resImg, cv::Size(480, 640));
		imshow("resImg", resImg);
		cv::waitKey(500);
		flag_ = true;
	}

	void QRcodeDetector::connectCallback()
	{
		if (!QR_code_subscriber_ && QR_code_publisher_.getNumSubscribers() > 0)
		{
			NODELET_INFO("Connecting to barcode topic.");
			QR_code_subscriber_ = nh_.subscribe<std_msgs::String>("/barcode", 10, &QRcodeDetector::Callback, this);
		}
	}

	void QRcodeDetector::disconnectCallback()
	{
		if (QR_code_publisher_.getNumSubscribers() == 0)
		{
			NODELET_INFO("Unsubscribing from barcode topic.");
			QR_code_subscriber_.shutdown();
		}
	}
}
