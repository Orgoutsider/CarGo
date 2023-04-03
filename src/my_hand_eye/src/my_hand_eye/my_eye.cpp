#include <my_hand_eye/my_eye.h>

namespace my_hand_eye
{
    MyEye::MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh) : arm_controller_(nh, pnh)
    {
		it_ = std::shared_ptr<image_transport::ImageTransport>(
      			new image_transport::ImageTransport(nh));
        std::string transport_hint;
    	pnh.param<std::string>("transport_hint", transport_hint, "raw");
    	camera_image_subscriber_ =
        	it_->subscribe<MyEye>("image_rect", 3, &MyEye::image_callback, this, image_transport::TransportHints(transport_hint));
		pnh.param<bool>("show_detections", arm_controller_.show_detections_, false);
        if (arm_controller_.show_detections_)
		{
			ROS_INFO("show debug image...");
            debug_image_publisher_ = nh.advertise<sensor_msgs::Image>("/detection_debug_image", 1);
		}
    }

    void MyEye::image_callback(const sensor_msgs::ImageConstPtr& image_rect)
    {
        // 目标跟踪
		static bool stop = false;
		int color = color_green;
		sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		if (!stop)
		{
			double u, v;
			arm_controller_.track(image_rect, color, tracker_camshift, u, v, stop, debug_image);
			if (arm_controller_.show_detections_)
				debug_image_publisher_.publish(debug_image);
		}
		else
		{
			// // 中间点抓取
			// static bool finish = false;
			// if (!finish)
			// {
			// 	double u, v;
			// 	arm_controller_.catch_straightly(image_rect, color, arm_controller_.z_turntable, finish, debug_image, true);
			// 	if (arm_controller_.show_detections_)
			// 		debug_image_publisher_.publish(debug_image);
			// }			
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
		// 	arm_controller_.put_with_ellipse(image_rect, color_green, 0, finish, debug_image);
		// 	if (arm_controller_.show_detections_)
		// 		debug_image_publisher_.publish(debug_image);
		// }
    }
} // namespace my_hand_eye
