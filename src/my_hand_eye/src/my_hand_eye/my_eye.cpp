#include <my_hand_eye/my_eye.h>

namespace my_hand_eye
{
	MyEye::MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh)
		: arm_controller_(nh, pnh),
		  as_(nh, "Arm", boost::bind(&MyEye::execute_callback, this, _1), false)
	{
		it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh));
		pnh.param<std::string>("transport_hint", transport_hint_, "raw");
		pnh.param<bool>("show_detections", arm_controller_.show_detections_, false);
		bool if_detect_QR_code = pnh.param<bool>("if_detect_QR_code", true);
		if (if_detect_QR_code)
			task_subscriber_ = nh.subscribe<my_hand_eye::ArrayofTaskArrays>(
				"/task", 10, &MyEye::task_callback, this);
		else
			camera_image_subscriber_ =
				it_->subscribe<MyEye>("image_rect", 1, &MyEye::image_callback, this, image_transport::TransportHints(transport_hint_));
		if (arm_controller_.show_detections_)
		{
			ROS_INFO("show debug image...");
			debug_image_publisher_ = nh.advertise<sensor_msgs::Image>("/detection_debug_image", 1);
		}
		as_.registerPreemptCallback(boost::bind(&MyEye::preempt_callback, this));
		as_.start();
	}

	void MyEye::task_callback(const my_hand_eye::ArrayofTaskArraysConstPtr &task)
	{
		tasks_ = task;
		ROS_INFO_STREAM(
			"Get tasks:" << unsigned(tasks_->loop[0].task[0]) << unsigned(tasks_->loop[0].task[1]) << unsigned(tasks_->loop[0].task[2]) << "+"
						 << unsigned(tasks_->loop[1].task[0]) << unsigned(tasks_->loop[1].task[1]) << unsigned(tasks_->loop[1].task[2]));
		camera_image_subscriber_ =
			it_->subscribe<MyEye>("image_rect", 1, &MyEye::image_callback, this, image_transport::TransportHints(transport_hint_));
		task_subscriber_.shutdown();
	}

	void MyEye::image_callback(const sensor_msgs::ImageConstPtr &image_rect)
	{
		// 目标跟踪
		// static bool stop = false;
		// int color = color_green;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!stop)
		// {
		// 	double u, v;
		// 	arm_controller_.track(image_rect, color, tracker_camshift, u, v, stop, debug_image);
		// 	if (arm_controller_.show_detections_)
		// 		debug_image_publisher_.publish(debug_image);
		// }
		// else
		// {
		// // 中间点抓取
		// static bool finish = false;
		// if (!finish)
		// {
		// 	double u, v;
		// 	arm_controller_.catch_straightly(image_rect, color, arm_controller_.z_turntable, finish, debug_image, true);
		// 	if (arm_controller_.show_detections_)
		// 		debug_image_publisher_.publish(debug_image);
		// }
		// }

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

		// 椭圆识别
		static bool finish = false;
		sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		if (!finish)
		{
			double u, v;
			arm_controller_.put_with_ellipse(image_rect, color_green, 0, finish, debug_image);
			if (arm_controller_.show_detections_)
				debug_image_publisher_.publish(debug_image);
		}
	}

	void MyEye::execute_callback(const ArmGoalConstPtr &goal)
	{
		if (goal->route != goal->route_parking_area)
			ros::Duration(2).sleep();
		ArmResult ar;
		as_.setSucceeded(ar, "Arm finish tasks");
	}

	void MyEye::preempt_callback()
	{
		ROS_ERROR("Arm Preempt Requested!");
		// 不正常现象，需要停止所有与底盘相关的联系，避免后续受到影响
		as_.setPreempted(ArmResult(), "Got preempted by a new goal");
	}
} // namespace my_hand_eye
