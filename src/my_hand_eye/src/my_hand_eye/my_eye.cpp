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
		pnh.param<bool>("show_detections", arm_controller_.show_detections, false);
		bool if_detect_QR_code = pnh.param<bool>("if_detect_QR_code", true);
		if (if_detect_QR_code)
			task_subscriber_ = nh.subscribe<my_hand_eye::ArrayofTaskArrays>(
				"/task", 10, &MyEye::task_callback, this);
		else
			camera_image_subscriber_ =
				it_->subscribe<MyEye>("image_rect", 1, &MyEye::image_callback, this, image_transport::TransportHints(transport_hint_));
		if (arm_controller_.show_detections)
		{
			ROS_INFO("show debug image...");
			debug_image_publisher_ = nh.advertise<sensor_msgs::Image>("/detection_debug_image", 1);
		}
		as_.registerPreemptCallback(boost::bind(&MyEye::preempt_callback, this));
		as_.start();
		dr_server_.setCallback(boost::bind(&MyEye::dr_callback, this, _1, _2));
	}

	void MyEye::task_callback(const my_hand_eye::ArrayofTaskArraysConstPtr &task)
	{
		tasks_ = task;
		ROS_INFO_STREAM(
			"Get tasks:" << unsigned(tasks_->loop[0].task[0]) << unsigned(tasks_->loop[0].task[1]) << unsigned(tasks_->loop[0].task[2]) << "+"
						 << unsigned(tasks_->loop[1].task[0]) << unsigned(tasks_->loop[1].task[1]) << unsigned(tasks_->loop[1].task[2]));
		camera_image_subscriber_ =
			it_->subscribe<MyEye>("image_rect", 1, &MyEye::image_callback, this, image_transport::TransportHints(transport_hint_));
	}

	void MyEye::image_callback(const sensor_msgs::ImageConstPtr &image_rect)
	{
		// // 输出检测物料位置
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_position(image_rect, arm_controller_.z_turntable, color_green, debug_image, true);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

		// 外参校正
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_extrinsics_correction(image_rect, 6.79367, 25.4134, arm_controller_.z_turntable, color_green, debug_image);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

		// 目标跟踪
		static bool first = true;
		int color = color_blue;
		sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		double x, y;
		arm_controller_.track(image_rect, color, first, x, y, debug_image);
		if (arm_controller_.show_detections)
			debug_image_publisher_.publish(debug_image);
		// 
		// 直接抓取
		// static bool finish = false;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!finish)
		// {
		// 	double u, v;
		// 	arm_controller_.catch_straightly(image_rect, color_red, arm_controller_.z_turntable, finish, debug_image, true, false);
		// 	if (arm_controller_.show_detections)
		// 		debug_image_publisher_.publish(debug_image);
		// }

		// 椭圆识别
		// static bool finish = false;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!finish)
		// {
		// 	double u, v;
		// 	arm_controller_.put_with_ellipse(image_rect, color_green, 0, finish, debug_image);
		// 	if (arm_controller_.show_detections)
		// 		debug_image_publisher_.publish(debug_image);
		// }

		// 边界线查找
		// static bool finish = false;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!finish)
		// {
		// 	double distance, yaw;
		// 	arm_controller_.find_border(image_rect, distance, yaw, finish, debug_image);
		// 	if (arm_controller_.show_detections)
		// 		debug_image_publisher_.publish(debug_image);
		// }

		// 停车区查找
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// double x = 0, y = 0;
		// arm_controller_.find_parking_area(image_rect, x, y, debug_image);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);
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

	void MyEye::dr_callback(drConfig &config, uint32_t level)
	{
		if (arm_controller_.z_parking_area != config.z_parking_area)
			arm_controller_.z_parking_area = config.z_parking_area;
		if (arm_controller_.threshold != config.threshold)
			arm_controller_.threshold = config.threshold;
	}
} // namespace my_hand_eye
