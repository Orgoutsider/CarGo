#include "my_hand_eye/QRcode_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(my_hand_eye::QRcodeDetector, nodelet::Nodelet);

namespace my_hand_eye
{
	void QRcodeDetector::onInit()
	{
		ros::NodeHandle &nh = getNodeHandle();
		ros::NodeHandle &pnh = getPrivateNodeHandle();
		bool if_detect_QR_code = pnh.param<bool>("if_detect_QR_code", false);
    	std::string transport_hint;
    	pnh.param<std::string>("transport_hint", transport_hint, "raw");

		if (if_detect_QR_code)
		{
			QR_code_subscriber_ = nh.subscribe<std_msgs::String>("/barcode", 5, &QRcodeDetector::Callback, this);
			QR_code_publisher_ = nh.advertise<my_hand_eye::ArrayofTaskArrays>("/task", 10);
			flag_ = false;
		}
		arm_controller_.init(nh, pnh, false);
		it_ = std::shared_ptr<image_transport::ImageTransport>(
      			new image_transport::ImageTransport(nh));
    	camera_image_subscriber_ =
        	it_->subscribe<QRcodeDetector>("image_rect", 1, &QRcodeDetector::imageCallback, this, image_transport::TransportHints(transport_hint));
		pnh.param<bool>("show_detections", arm_controller_.show_detections_, false);
        if (arm_controller_.show_detections_)
		{
			NODELET_INFO("show debug image...");
            debug_image_publisher_ = nh.advertise<sensor_msgs::Image>("/detection_debug_image", 1);
		}
	}

	void QRcodeDetector::Callback(const std_msgs::StringConstPtr &info)
	{
		if (flag_)
			return;
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
				putText(resImg, txt_temp, cv::Point(ptx_info[ii] + txt_Xoffset, pty_info[ii] + txt_Yoffset), cv::FONT_HERSHEY_PLAIN, txt_size, cv::Scalar::all(255), txt_thick, cv::FILLED, false);
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
				task =  num[ii++] - '0';
				tarr.task[i] = task;
			}
			arr.loop.push_back(tarr);		
		}
		NODELET_INFO("Succeeded to detect QR Code!");
		QR_code_publisher_.publish(arr);
		cv::namedWindow("resImg", cv::WINDOW_NORMAL);
		cv::setWindowProperty("resImg", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		cv::resize(resImg, resImg, cv::Size(2240, 1680));
		imshow("resImg", resImg);
		cv::waitKey(500);
		flag_ = true;
	}

	void QRcodeDetector::imageCallback(const sensor_msgs::ImageConstPtr& image_rect)
	{
		// 目标跟踪
		static bool stop = false;
		sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		if (!stop)
		{
			double u, v;
			arm_controller_.target_tracking(image_rect, color_blue, u, v, stop, debug_image);
			if (arm_controller_.show_detections_)
				debug_image_publisher_.publish(debug_image);
		}

		// // 中间点抓取
		// static bool finish = false;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!finish)
		// {
		// 	double u, v;
		// 	arm_controller_.catch_straightly(image_rect, color_red, arm_controller_.z_turntable, finish, debug_image, true);
		// 	if (arm_controller_.show_detections_)
		// 		debug_image_publisher_.publish(debug_image);
		// }

		// // 椭圆识别
		// static bool finish = false;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!finish)
		// {
		// 	double u, v;
		// 	arm_controller_.put_with_ellipse(image_rect, color_blue, 0, finish, debug_image);
		// 	if (arm_controller_.show_detections_)
		// 		debug_image_publisher_.publish(debug_image);
		// }
	}
}
