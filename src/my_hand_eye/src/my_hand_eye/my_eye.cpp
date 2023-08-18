#include <my_hand_eye/my_eye.h>

namespace my_hand_eye
{
	MyEye::MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh)
		: arm_controller_(nh, pnh),
		  as_(nh, "Arm", false), finish_adjusting_(true), finish_(true), task_idx_(0)
	{
		it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh));
		pnh.param<std::string>("transport_hint", transport_hint_, "raw");
		pnh.param<bool>("show_detections", arm_controller_.show_detections, false);
		pnh.param<bool>("debug", debug_, false);
		if (debug_)
			dr_server_.setCallback(boost::bind(&MyEye::dr_callback, this, _1, _2));
		else
			ROS_INFO("my_hand_eye_do_node: don't modify paramater");
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

	void MyEye::next_task()
	{
		if ((++task_idx_) >= 3)
			task_idx_ = 0;
	}

	Color MyEye::which_color(bool next) const
	{
		switch (tasks_.loop[arm_goal_.loop].task[(task_idx_ + next) >= 3 ? 0 : (task_idx_ + next)])
		{
		case color_red:
			return color_red;

		case color_green:
			return color_green;

		case color_blue:
			return color_blue;

		default:
			ROS_ERROR("which_color: Color error!");
			return color_red;
		}
	}

	void MyEye::task_callback(const my_hand_eye::ArrayofTaskArraysConstPtr &task)
	{
		tasks_ = *task;
		ROS_INFO_STREAM(
			"Get tasks:" << unsigned(tasks_.loop[0].task[0]) << unsigned(tasks_.loop[0].task[1]) << unsigned(tasks_.loop[0].task[2]) << "+"
						 << unsigned(tasks_.loop[1].task[0]) << unsigned(tasks_.loop[1].task[1]) << unsigned(tasks_.loop[1].task[2]));
		camera_image_subscriber_ =
			it_->subscribe<MyEye>("image_rect", 1, &MyEye::image_callback, this, image_transport::TransportHints(transport_hint_));
	}

	void MyEye::image_callback(const sensor_msgs::ImageConstPtr &image_rect)
	{
		if (!as_.isActive())
			return;
		sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		bool valid = false;
		switch (arm_goal_.route)
		{
		case arm_goal_.route_rest:
			return;

		case arm_goal_.route_raw_material_area:
			valid = operate_center(image_rect, debug_image);
			break;

		case arm_goal_.route_roughing_area:
			valid = operate_ellipse(image_rect, debug_image);
			break;

		case arm_goal_.route_semi_finishing_area:
			valid = operate_ellipse(image_rect, debug_image);
			break;

		case arm_goal_.route_parking_area:
			break;

		case arm_goal_.route_border:
			break;

		default:
			return;
		}
		if (arm_controller_.show_detections)
			debug_image_publisher_.publish(debug_image);

		// 输出检测物料位置
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_cargo(image_rect, color_blue, arm_controller_.z_ellipse, debug_image, false, true);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

		// 外参校正
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_extrinsics_correction(image_rect, -6.18287, 20.3357, arm_controller_.z_turntable, color_blue, debug_image);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);
		//
		// 直接抓取
		// static bool finish = false;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!finish)
		// {
		// 	double u, v;
		// 	arm_controller_.catch_straightly(image_rect, color_red, finish, debug_image, false, false);
		// 	if (arm_controller_.show_detections)
		// 		debug_image_publisher_.publish(debug_image);
		// }

		// 椭圆识别
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_ellipse(image_rect, color_green, debug_image, true);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

		// z校正
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_z_correction(image_rect, {0.15, -0.65}, {0.14, -0.5}, {0, -0.46}, debug_image);
		// if (arm_controller_.show_detections)
		// 	debug_image_publisher_.publish(debug_image);

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
		// if (as_.isPreemptRequested() || !ros::ok())
		// {
		// 	ROS_ERROR("Arm Preempt Requested!");
		// 	cancel_all();
		// 	as_.setPreempted(ArmResult(), "Got preempted by a new goal");
		// 	return;
		// }
		arm_goal_ = *(as_.acceptNewGoal());
		switch (arm_goal_.route)
		{
		case arm_goal_.route_border:
			arm_controller_.target_pose.target = arm_controller_.target_pose.target_border;
			break;

		case arm_goal_.route_raw_material_area:
			arm_controller_.target_pose.target = arm_controller_.target_pose.target_center;
			break;

		case arm_goal_.route_roughing_area:
			arm_controller_.target_pose.target = arm_controller_.target_pose.target_ellipse;
			break;

		case arm_goal_.route_semi_finishing_area:
			arm_controller_.target_pose.target = arm_controller_.target_pose.target_ellipse;
			break;

		case arm_goal_.route_parking_area:
			arm_controller_.target_pose.target = arm_controller_.target_pose.target_parking_area;
			break;

		default:
			break;
		}
	}

	void MyEye::preempt_callback()
	{
		ROS_ERROR("Arm Preempt Requested!");
		// 不正常现象，需要停止所有与底盘相关的联系，避免后续受到影响
		cancel_all();
		as_.setPreempted(ArmResult(), "Got preempted by a new goal");
	}

	void MyEye::cancel_all()
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
				msg.header.frame_id = "base_footprint";
				msg.header.stamp = ros::Time::now() - ros::Duration(0.2);
				pose_publisher_.publish(msg);
			}
			finish_adjusting_ = true;
		}
		if (!finish_)
			finish_ = true;
		if (arm_goal_.route != arm_goal_.route_rest)
			arm_goal_.route = arm_goal_.route_rest;
	}

	bool MyEye::operate_center(const sensor_msgs::ImageConstPtr &image_rect,
							   sensor_msgs::ImagePtr &debug_image)
	{
		if (finish_)
		{
			finish_adjusting_ = false;
			finish_ = false;
			ROS_INFO("Start to operate center...");
		}
		bool valid = true;
		if (!finish_adjusting_)
		{
			// static int err_cnt = 0;
			Pose2DMightEnd msg;
			msg.end = false;
			valid = arm_controller_.find_cargo(image_rect, msg, debug_image, false);
			if (valid)
			{
				if (msg.end)
				{
					finish_adjusting_ = true;
				}
				ArmFeedback feedback;
				feedback.pme = msg;
				as_.publishFeedback(feedback);
				return valid;
			}
			else
			{
				// 发送此数据表示车辆即刻停止，重新寻找定位物体
				msg.pose.x = msg.not_change;
				msg.pose.y = msg.not_change;
				msg.pose.theta = msg.not_change;
				ArmFeedback feedback;
				feedback.pme = msg;
				as_.publishFeedback(feedback);
			}
		}
		else if (!debug_)
		{
			double x = 0, y = 0;
			static bool first = true;
			bool finish = false;
			valid = arm_controller_.track(image_rect, which_color(), first, x, y, debug_image) &&
					arm_controller_.catch_after_tracking(x, y, which_color(false), which_color(true),
														 task_idx_ == 2, finish);
			if (finish)
			{
				next_task();
				if (task_idx_ == 0)
				{
					first = true;
					finish_ = true;
					arm_controller_.catched = false;
					arm_goal_.route = arm_goal_.route_rest;
					as_.setSucceeded(ArmResult(), "Arm finish tasks");
					ROS_INFO("Finish operating center...");
				}
			}
		}
		else
		{
			static bool first = true;
			double x = 0, y = 0;
			valid = arm_controller_.track(image_rect, color_green, first, x, y, debug_image);
		}
		return valid;
	}

	bool MyEye::operate_ellipse(const sensor_msgs::ImageConstPtr &image_rect,
								sensor_msgs::ImagePtr &debug_image)
	{
		if (finish_)
		{
			finish_adjusting_ = false;
			finish_ = false;
			ROS_INFO("Start to operate ellipse...");
		}
		bool valid = true;
		Pose2DMightEnd msg;
		static ros::Time time_stop;
		if (!finish_adjusting_)
		{
			msg.end = false;
			if (arm_goal_.loop == 0 || arm_goal_.route == arm_goal_.route_roughing_area)
				valid = arm_controller_.find_ellipse(image_rect, msg, debug_image, false);
			else if (arm_goal_.loop == 1 && arm_goal_.route == arm_goal_.route_semi_finishing_area)
				valid = arm_controller_.find_cargo(image_rect, msg, debug_image, true);
			else
			{
				ROS_ERROR("Invalid loop!");
				return false;
			}
			if (valid)
			{
				pose_publisher_.publish(msg);
				if (msg.end)
				{
					ROS_INFO("x:%lf y:%lf theta:%lf", msg.pose.x, msg.pose.y, msg.pose.theta);
					finish_adjusting_ = true;
					ArmFeedback feedback;
					feedback.pme = msg;
					as_.publishFeedback(feedback);
					time_stop = ros::Time::now() + ros::Duration(0.1);
				}
			}
			else
			{
				// 发送此数据表示车辆即刻停止，重新寻找定位物体
				msg.pose.x = msg.not_change;
				msg.pose.y = msg.not_change;
				msg.pose.theta = msg.not_change;
				msg.end = false;
				msg.header = image_rect->header;
				msg.header.frame_id = "base_footprint";
				pose_publisher_.publish(msg);
				finish_adjusting_ = false;
			}
		}
		else if (!debug_)
		{
			if ((image_rect->header.stamp - time_stop).toSec() < 0)
				return valid;
			if (arm_goal_.loop == 0 || arm_goal_.loop == 1)
			{
				msg.end = true;
				bool pal = (arm_goal_.loop == 1 && arm_goal_.route == arm_goal_.route_semi_finishing_area);
				bool fin = pal ? arm_controller_.find_cargo(image_rect, msg, debug_image, true, true)
							   : arm_controller_.find_ellipse(image_rect, msg, debug_image, true);
				if (fin)
				{
					arm_controller_.put(which_color(), pal, false);
					next_task();
					arm_controller_.put(which_color(), pal, false);
					next_task();
					arm_controller_.put(which_color(), pal, arm_goal_.route == arm_goal_.route_semi_finishing_area);
					next_task();
					if (arm_goal_.route == arm_goal_.route_roughing_area)
					{
						arm_controller_.catch_after_putting(which_color(), false);
						next_task();
						arm_controller_.catch_after_putting(which_color(), false);
						next_task();
						arm_controller_.catch_after_putting(which_color(), true);
						next_task();
					}
					finish_ = true;
					arm_goal_.route = arm_goal_.route_rest;
					as_.setSucceeded(ArmResult(), "Arm finish tasks");
					ROS_INFO("Finish operating ellipse...");
				}
				else if (!msg.end)
				{
					finish_adjusting_ = false;
				}
			}
			else
			{
				ROS_ERROR("Invalid loop!");
				return false;
			}
		}
		else
		{
			finish_ = true;
			arm_goal_.route = arm_goal_.route_rest;
			as_.setSucceeded(ArmResult(), "Arm finish tasks");
			ROS_INFO("Finish operating ellipse...");
		}
		return valid;
	}

	void MyEye::dr_callback(drConfig &config, uint32_t level)
	{
		if (arm_controller_.z_parking_area != config.z_parking_area)
			arm_controller_.z_parking_area = config.z_parking_area;
		if (arm_controller_.threshold != config.threshold)
			arm_controller_.threshold = config.threshold;
		Action a;
		if (arm_controller_.target_pose.target == arm_controller_.target_pose.target_ellipse ||
			arm_controller_.target_pose.target == arm_controller_.target_pose.target_border)
			a = Action(config.target_x, config.target_y, 0).front2left().arm2footprint();
		else
			a = Action(config.target_x, config.target_y, 0).arm2footprint();
		if (arm_controller_.target_pose.pose[arm_controller_.target_pose.target].x != a.x)
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].x = a.x;
		if (arm_controller_.target_pose.pose[arm_controller_.target_pose.target].y != a.y)
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].y = a.y;
		if (arm_controller_.target_pose.pose[arm_controller_.target_pose.target].theta !=
			Angle(config.target_theta_deg).rad())
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].theta =
				Angle(config.target_theta_deg).rad();
	}
} // namespace my_hand_eye
