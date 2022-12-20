#include "my_hand_eye/color_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(eye::ColorDetector, nodelet::Nodelet);

namespace eye
{
	void ColorDetector::onInit()
	{
		ros::NodeHandle &nh = getNodeHandle();
		ros::NodeHandle &pnh = getPrivateNodeHandle();
		it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh));

		std::string transport_hint;
		pnh.param<std::string>("transport_hint", transport_hint, "raw");
		pnh.getParam("task", task_);

		camera_image_subscriber_ =
			it_->subscribe("image_rect", 1,
						   &ColorDetector::imageCallback, this,
						   image_transport::TransportHints(transport_hint));
	}

	void ColorDetector::imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
	{
		unsigned RGB = 0;
		switch (task_)
		{
		case 0:
			return;
		case 1:
			RGB = red;
			break;
		case 2:
			RGB = green;
			break;
		case 3:
			RGB = blue;
			break;
		default:
			ROS_ERROR("Task code must be set to 0,1,2,3!");
			break;
		}
		try
		{
			cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		if (cv_image_->image.empty())
		{
			ROS_ERROR("No data!");
			return;
		}
		cv::Mat srcGauss, srcHSV;
		cv::GaussianBlur(cv_image_->image, srcGauss, cv::Size(7, 7), 0, 0);
		cv::cvtColor(srcGauss, srcHSV, cv::COLOR_BGR2HSV);
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(25, 25));
		cv::inRange(srcHSV, Low[RGB], Up[RGB], cv_image_->image);
		cv::erode(cv_image_->image, cv_image_->image, element);

		std::vector<std::vector<cv::Point>> contours;									  //轮廓容器
		cv::Point2f vtx[4];													  //矩形顶点容器
		cv::findContours(cv_image_->image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); //查找轮廓
		cv::Mat dst = cv::Mat::zeros(cv_image_->image.size(), CV_8UC3);					  //创建空白图像
		std::vector<cv::Point> figure_info;
		for (int i = 0; i < contours.size(); i++)
		{
			cv::Scalar colors = cv::Scalar(rngs_.uniform(0, 255), rngs_.uniform(0, 255), rngs_.uniform(0, 255)); //创建随机颜色，便于区分
			cv::drawContours(dst, contours, i, colors, 1);												  //随机颜色绘制轮廓
			cv::RotatedRect rect = cv::minAreaRect(contours[i]);											  //求解最小矩阵
			rect.points(vtx);																		  //确定旋转矩阵的四个顶点
			std::cout << "x:" << (vtx[0].x + vtx[2].x) / 2 << std::endl;
			std::cout << "y:" << (vtx[0].y + vtx[2].y) / 2 << std::endl;
			figure_info.push_back(cv::Point((vtx[0].x + vtx[2].x) / 2, (vtx[0].y + vtx[2].y) / 2));
			// (*rects).push_back(rect);
			// for (int j = 0; j < 4; j++)
			// {
			// 	line(dst, vtx[j], vtx[(j + 1) % 4], colors, 2); //随机颜色绘制矩形
			// }
		}
	}

}
