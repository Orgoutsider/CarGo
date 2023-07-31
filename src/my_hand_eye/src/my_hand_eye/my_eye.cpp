#include <my_hand_eye/my_eye.h>

namespace my_hand_eye
{
	MyEye::MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh)
		: motor_status_(false), arm_controller_(nh, pnh),
		  as_(nh, "Arm", false)
	{
		it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh));
		pnh.param<std::string>("transport_hint", transport_hint_, "raw");
		pnh.param<bool>("show_detections", arm_controller_.show_detections, false);
		pnh.param<bool>("param_modification", param_modification_, false);
		if (param_modification_)
			dr_server_.setCallback(boost::bind(&MyEye::dr_callback, this, _1, _2));
		bool if_detect_QR_code = pnh.param<bool>("if_detect_QR_code", true);
		if (if_detect_QR_code)
			task_subscriber_ = nh.subscribe<my_hand_eye::ArrayofTaskArrays>(
				"/task", 10, &MyEye::task_callback, this);
		else
			camera_image_subscriber_ =
				it_->subscribe<MyEye>("image_rect", 1, &MyEye::image_callback, this, image_transport::TransportHints(transport_hint_));
		pose_publisher_ = nh.advertise<Pose2DMightEnd>("/vision_eye", 3);
		if (arm_controller_.show_detections)
		{
			ROS_INFO("show debug image...");
			debug_image_publisher_ = nh.advertise<sensor_msgs::Image>("/detection_debug_image", 1);
		}
		as_.registerGoalCallback(boost::bind(&MyEye::goal_callback, this));
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
	}

	void MyEye::image_callback(const sensor_msgs::ImageConstPtr &image_rect)
	{
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// bool valid = false;
		// switch (arm_goal_.route)
		// {
		// case arm_goal_.route_rest:
		// 	ros::Duration(0.4).sleep();
		// 	break;

		// case arm_goal_.route_raw_material_area:
		// 	valid = operate_raw_material_area(image_rect, debug_image);
		// 	break;

		// default:
		// 	break;
		// }
		// if (valid && arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

		// // 输出检测物料位置
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_position(image_rect, arm_controller_.z_turntable, color_blue, debug_image, false);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

		// 外参校正
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_extrinsics_correction(image_rect, -6.18287, 20.3357, arm_controller_.z_turntable, color_blue, debug_image);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

		// 目标跟踪
		// static bool first = true;
		// Color color = color_blue;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// double x, y;
		// arm_controller_.track(image_rect, color, first, x, y, debug_image);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);
		//
		// 直接抓取
		static bool finish = false;
		sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		if (!finish)
		{
			double u, v;
			arm_controller_.catch_straightly(image_rect, color_blue, finish, debug_image, true, false);
			if (arm_controller_.show_detections)
				debug_image_publisher_.publish(debug_image);
		}

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

	void MyEye::goal_callback()
	{
		// if (goal->route != goal->route_parking_area)
		// 	ros::Duration(2).sleep();
		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_ERROR("Arm Preempt Requested!");
			stop_adjustment();
			if (arm_goal_.route != arm_goal_.route_rest)
				arm_goal_.route = arm_goal_.route_rest;
			as_.setPreempted(ArmResult(), "Got preempted by a new goal");
			return;
		}
		arm_goal_.route = as_.acceptNewGoal()->route;
		// as_.setSucceeded(ArmResult(), "Arm finish tasks");
	}

	void MyEye::preempt_callback()
	{
		ROS_ERROR("Arm Preempt Requested!");
		// 不正常现象，需要停止所有与底盘相关的联系，避免后续受到影响
		stop_adjustment();
		if (arm_goal_.route != arm_goal_.route_rest)
			arm_goal_.route = arm_goal_.route_rest;
		as_.setPreempted(ArmResult(), "Got preempted by a new goal");
	}

	void MyEye::stop_adjustment()
	{
		if (!finish_adjusting_)
		{
			if (arm_goal_.route == arm_goal_.route_raw_material_area)
			{
				ArmFeedback feedback;
				feedback.pme.end = true;
				as_.publishFeedback(feedback);
			}
			else
			{
				Pose2DMightEnd msg;
				msg.end = true;
				pose_publisher_.publish(msg);
			}
			finish_adjusting_ = true;
		}
	}

	bool MyEye::operate_raw_material_area(const sensor_msgs::ImageConstPtr &image_rect,
										  sensor_msgs::ImagePtr &debug_image)
	{
		static bool last_finish = true;
		// if (last_finish && arm_goal_)
		// {
		// 	finish_adjusting_ = false;
		// 	ROS_INFO("Start operate raw material area...");
		// }
		// else if (!last_finish && finish_)
		// {
		// 	if (!finish_adjusting_)
		// 		finish_adjusting_ = true;
		// 	last_finish = finish_;
		// 	if (param_modification_)
		// 		ROS_INFO("Finish operate raw material area...");
		// 	return true;
		// }
		// else if (finish_) // 前后一样完成就不用赋值了
		// {
		// 	ros::Duration(0.1).sleep();
		// 	return true;
		// }
		bool valid = true;
		static bool first = false;
		if (!finish_adjusting_)
		{
			static int err_cnt = 0;
			Pose2DMightEnd msg;
			msg.end = finish_adjusting_;
			valid = arm_controller_.find_center(image_rect, msg, debug_image);
			if (valid)
			{
				if (msg.end)
				{
					finish_adjusting_ = true;
				}
				pose_publisher_.publish(msg);
				if (err_cnt)
					err_cnt = 0;
			}
			else
			{
				// 发送此数据表示车辆即刻停止，重新寻找定位物体
				msg.pose.x = msg.not_change;
				msg.pose.y = msg.not_change;
				msg.pose.theta = msg.not_change;
				if ((++err_cnt) > 5)
				{
					finish_adjusting_ = true;
					msg.end = finish_adjusting_;
					ROS_WARN("Could not find 3 cargos. Try to catch...");
				}
				pose_publisher_.publish(msg);
			}
		}
		else if (!param_modification_)
		{
			// valid = arm_controller_.track(image_rect, )
		}
		// last_finish = finish_;
		return valid;
	}

	void MyEye::dr_callback(drConfig &config, uint32_t level)
	{
		if (arm_controller_.z_parking_area != config.z_parking_area)
			arm_controller_.z_parking_area = config.z_parking_area;
		if (arm_controller_.threshold != config.threshold)
			arm_controller_.threshold = config.threshold;
		if (arm_controller_.target_pose.pose[config.target].x != config.target_x)
			arm_controller_.target_pose.pose[config.target].x = config.target_x;
		if (arm_controller_.target_pose.pose[config.target].y != config.target_y)
			arm_controller_.target_pose.pose[config.target].y = config.target_y;
		if (arm_controller_.target_pose.pose[config.target].theta !=
			config.target_theta_deg / 180 * CV_PI)
			arm_controller_.target_pose.pose[config.target].theta =
				config.target_theta_deg / 180 * CV_PI;
		if (motor_status_ != config.motor_status)
		{
			motor_status_ = config.motor_status;
			if (motor_status_)
			{
				if (finish_adjusting_)
				{
					finish_adjusting_ = false;
				}
			}
			else
				stop_adjustment();
		}
	}
} // namespace my_hand_eye
