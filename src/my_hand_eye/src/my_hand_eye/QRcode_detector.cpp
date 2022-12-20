#include "my_hand_eye/QRcode_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(eye::QRcodeDetector, nodelet::Nodelet);

namespace eye
{
    void QRcodeDetector::onInit()
	{
		ros::NodeHandle &nh = getNodeHandle();
		ros::NodeHandle &pnh = getPrivateNodeHandle();
		it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh));

		std::string transport_hint;
		pnh.param<std::string>("transport_hint", transport_hint, "raw");

		camera_image_subscriber_ =
			it_->subscribe("image_rect", 1,
						   &QRcodeDetector::imageCallback, this,
						   image_transport::TransportHints(transport_hint));
	}

    void QRcodeDetector::imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
    {
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
    }
}
