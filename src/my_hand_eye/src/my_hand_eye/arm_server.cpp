#include "my_hand_eye/arm_server.h"

namespace my_hand_eye
{
	ArmServer::ArmServer()
		: pnh_("~"), arm_controller_(nh_, pnh_),
		  as_(nh_, "Arm", false), finish_adjusting_(true), finish_(true), task_idx_(0)
	{
		it_ = std::shared_ptr<image_transport::ImageTransport>(
			new image_transport::ImageTransport(nh_));
		pnh_.param<std::string>("transport_hint", transport_hint_, "raw");
		pnh_.param<bool>("show_detections", arm_controller_.show_detections, false);
		pnh_.param<bool>("debug", debug_, false);
		pnh_.param<int>("mode", mode_, debug_default);
		if (debug_)
			dr_server_.setCallback(boost::bind(&ArmServer::dr_callback, this, _1, _2));
		else
			ROS_INFO("my_hand_eye_do_node: don't modify paramater");
		arm_controller_.target_pose.debug = debug_;
		bool given_QR_code = pnh_.param<bool>("given_QR_code", true);
		if (given_QR_code)
			task_subscriber_ = nh_.subscribe<my_hand_eye::ArrayofTaskArrays>(
				"/task", 10, &ArmServer::task_callback, this);
		camera_image_subscriber_ =
			it_->subscribe<ArmServer>("image_rect", 1, &ArmServer::image_callback, this, image_transport::TransportHints(transport_hint_));
		pose_publisher_ = nh_.advertise<Pose2DMightEnd>("/vision_eye", 3);
		if (arm_controller_.show_detections)
		{
			ROS_INFO("show debug image...");
			debug_image_publisher_ = nh_.advertise<sensor_msgs::Image>("/detection_debug_image", 1);
		}
		done_server_ = nh_.advertiseService("moveDone", &ArmServer::done_callback, this);
		as_.registerGoalCallback(boost::bind(&ArmServer::goal_callback, this));
		as_.registerPreemptCallback(boost::bind(&ArmServer::preempt_callback, this));
		as_.start();
		arm_goal_.route = arm_goal_.route_rest;
	}

	void ArmServer::next_task()
	{
		if ((++task_idx_) >= 3)
			task_idx_ = 0;
	}

	Color ArmServer::which_color(bool next) const
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

	void ArmServer::task_callback(const my_hand_eye::ArrayofTaskArraysConstPtr &task)
	{
		if (arm_goal_.route == arm_goal_.route_QR_code_board)
		{
			if (!as_.isActive())
				return;
			as_.setSucceeded(ArmResult(), "Arm finish tasks");
			arm_goal_.route = arm_goal_.route_rest;
		}
		tasks_ = *task;
		ROS_INFO_STREAM(
			"Get tasks:" << unsigned(tasks_.loop[0].task[0]) << unsigned(tasks_.loop[0].task[1]) << unsigned(tasks_.loop[0].task[2]) << "+"
						 << unsigned(tasks_.loop[1].task[0]) << unsigned(tasks_.loop[1].task[1]) << unsigned(tasks_.loop[1].task[2]));
		task_subscriber_.shutdown();
		if (!camera_image_subscriber_)
			camera_image_subscriber_ =
				it_->subscribe<ArmServer>("image_rect", 1, &ArmServer::image_callback, this, image_transport::TransportHints(transport_hint_));
	}

	void ArmServer::image_callback(const sensor_msgs::ImageConstPtr &image_rect)
	{
		sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		switch (mode_)
		{
		case debug_default:
			if (arm_controller_.ready_yolo(image_rect) && task_subscriber_)
			{
				camera_image_subscriber_.shutdown();
				return;
			}
			if (!as_.isActive())
				return;
			switch (arm_goal_.route)
			{
			case arm_goal_.route_rest:
				return;

			case arm_goal_.route_QR_code_board:
				ROS_WARN_ONCE("When using QR code. Please set given_QR_code to true");
				return;

			case arm_goal_.route_raw_material_area:
				operate_center(image_rect, debug_image);
				break;

			case arm_goal_.route_roughing_area:
				operate_ellipse(image_rect, debug_image);
				break;

			case arm_goal_.route_semi_finishing_area:
				operate_ellipse(image_rect, debug_image);
				break;

			case arm_goal_.route_parking_area:
				operate_parking_area(image_rect, debug_image);
				break;

			case arm_goal_.route_border:
				operate_border(image_rect, debug_image);
				break;

			default:
				return;
			}
			break;

		case debug_cargo:
			// 输出检测物料位置
			arm_controller_.log_cargo(image_rect, color_blue, arm_controller_.z_turntable,
									  debug_image, true, false);
			break;

		case debug_ellipse:
			arm_controller_.log_ellipse(image_rect, color_blue, debug_image, true);
			break;

		case debug_border:
			arm_controller_.log_border(image_rect, debug_image);
			break;

		case debug_parking_area:
			arm_controller_.log_parking_area(image_rect, debug_image);
			break;

		default:
			ROS_WARN("Invalid mode %d", mode_);
			break;
		}
		if (arm_controller_.show_detections && debug_image->height)
			debug_image_publisher_.publish(debug_image);

		// 外参校正
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_extrinsics_correction(image_rect, 2.59692, 26.0293, arm_controller_.z_turntable, color_green, debug_image);
		// if (arm_controller_.show_detections && debug_image->height)
		// 	debug_image_publisher_.publish(debug_image);

		// 直接抓取
		// static bool finish = false;
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// if (!finish)
		// {
		// 	double u, v;
		// 	arm_controller_.catch_straightly(image_rect, color_red, finish, debug_image, false, false);
		// 	if (arm_controller_.show_detections && debug_image->height)
		// 		debug_image_publisher_.publish(debug_image);
		// }

		// z校正
		// sensor_msgs::ImagePtr debug_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());
		// arm_controller_.log_z_correction(image_rect, {0.15, -0.65}, {0.14, -0.5}, {0, -0.46}, debug_image);
		// if (arm_controller_.show_detections && debug_image->height)
		// 	debug_image_publisher_.publish(debug_image);
	}

	void ArmServer::goal_callback()
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

		case arm_goal_.route_QR_code_board:
			arm_controller_.ready(arm_goal_.left_ready);
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

	void ArmServer::preempt_callback()
	{
		ROS_WARN("Arm Preempt Requested!");
		// 需要停止所有与底盘相关的联系，避免后续受到影响
		cancel_all();
		as_.setPreempted(ArmResult(), "Got preempted by a new goal");
	}

	void ArmServer::cancel_all()
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
				if (arm_goal_.route == arm_goal_.route_border)
				{
					ArmFeedback feedback;
					feedback.pme.end = true;
					as_.publishFeedback(feedback);
				}
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

	bool ArmServer::operate_center(const sensor_msgs::ImageConstPtr &image_rect,
								   sensor_msgs::ImagePtr &debug_image)
	{
		static ros::Time err_time;
		if (finish_)
		{
			finish_adjusting_ = false;
			finish_ = false;
			ROS_INFO("Start to operate center...");
			err_time = image_rect->header.stamp;
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
				err_time = image_rect->header.stamp;
				return valid;
			}
			else
			{
				// 发送此数据表示车辆即刻停止，重新寻找定位物体
				msg.pose.x = msg.not_change;
				msg.pose.y = msg.not_change;
				msg.pose.theta = msg.not_change;
				msg.header = image_rect->header;
				msg.header.frame_id = "base_footprint";
				if ((image_rect->header.stamp - err_time).toSec() > 13) // 超时
				{
					finish_adjusting_ = true;
					finish_ = true;
					ArmFeedback feedback;
					msg.end = true;
					feedback.pme = msg;
					as_.publishFeedback(feedback);
					// feedback与done间隔一段时间
					ros::Duration(0.2).sleep();
					// 结束该函数
					arm_controller_.find_cargo(image_rect, msg, debug_image, false);
					as_.setAborted(ArmResult(), "Arm finish tasks");
					arm_controller_.finish_catching();
					arm_goal_.route = arm_goal_.route_rest;
					ROS_WARN("Failed to operating center.");
				}
				else
				{
					ArmFeedback feedback;
					feedback.pme = msg;
					as_.publishFeedback(feedback);
				}
			}
		}
		else if (!debug_)
		{
			double x = 0, y = 0;
			static bool first = true;
			bool finish = false;
			valid = arm_controller_.track(image_rect, which_color(), first, x, y, debug_image) &&
					arm_controller_.catch_after_tracking(x, y, which_color(false), which_color(true),
														 false, finish);
			if (finish)
			{
				next_task();
				if (task_idx_ == 0)
				{
					first = true;
					finish_ = true;
					as_.setSucceeded(ArmResult(), "Arm finish tasks");
					arm_controller_.finish_catching();
					arm_goal_.route = arm_goal_.route_rest;
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

	bool ArmServer::operate_ellipse(const sensor_msgs::ImageConstPtr &image_rect,
									sensor_msgs::ImagePtr &debug_image)
	{
		static ros::Time err_time;
		if (finish_)
		{
			finish_adjusting_ = false;
			finish_ = false;
			ROS_INFO("Start to operate ellipse...");
			err_time = image_rect->header.stamp;
		}
		bool valid = true;
		Pose2DMightEnd msg;
		if (!finish_adjusting_)
		{
			msg.end = false;
			if (arm_goal_.loop == 0 || arm_goal_.route == arm_goal_.route_roughing_area)
			{
				if (!debug_ && arm_goal_.route == arm_goal_.route_semi_finishing_area)
					ROS_WARN_ONCE("If you want to palletize, set loop to 1.");
				valid = arm_controller_.find_ellipse(image_rect, msg, debug_image, false);
			}
			else if (arm_goal_.loop == 1 && arm_goal_.route == arm_goal_.route_semi_finishing_area)
				valid = arm_controller_.find_cargo(image_rect, msg, debug_image, true, false);
			else
			{
				ROS_ERROR("Invalid loop!");
				return false;
			}
			if (valid)
			{
				pose_publisher_.publish(msg);
				err_time = image_rect->header.stamp;
				if (msg.end)
				{
					ROS_INFO("x:%lf y:%lf theta:%lf", msg.pose.x, msg.pose.y, msg.pose.theta);
					finish_adjusting_ = true;
					// ros::Duration(0.2).sleep();
					ArmFeedback feedback;
					feedback.pme = msg;
					as_.publishFeedback(feedback);
					arm_controller_.theta_turn = 0;
				}
			}
			else
			{
				// 发送此数据表示车辆即刻停止，重新寻找定位物体
				msg.pose.x = msg.not_change;
				msg.pose.y = msg.not_change;
				msg.pose.theta = msg.not_change;
				msg.header = image_rect->header;
				msg.header.frame_id = "base_footprint";
				if ((image_rect->header.stamp - err_time).toSec() > 5) // 超时
				{
					finish_adjusting_ = true;
					finish_ = true;
					ArmFeedback feedback;
					msg.end = true;
					feedback.pme = msg;
					as_.publishFeedback(feedback);
					// 结束该函数
					if (arm_goal_.loop == 0 || arm_goal_.route == arm_goal_.route_roughing_area)
						arm_controller_.find_ellipse(image_rect, msg, debug_image, false);
					else if (arm_goal_.loop == 1 && arm_goal_.route == arm_goal_.route_semi_finishing_area)
						arm_controller_.find_cargo(image_rect, msg, debug_image, true, false);
					arm_controller_.theta_turn = 0;
					bool pal = (arm_goal_.loop == 1 && arm_goal_.route == arm_goal_.route_semi_finishing_area);
					arm_controller_.put(which_color(), pal, false);
					next_task();
					arm_controller_.put(which_color(), pal, false);
					next_task();
					arm_controller_.put(which_color(), pal, arm_goal_.route == arm_goal_.route_semi_finishing_area);
					next_task();
					if (arm_goal_.route == arm_goal_.route_roughing_area)
					{
						arm_controller_.ready_after_putting();
						arm_controller_.catch_after_putting(which_color(), false);
						next_task();
						arm_controller_.catch_after_putting(which_color(), false);
						next_task();
						arm_controller_.catch_after_putting(which_color(), true);
						next_task();
					}
					if (arm_goal_.route == arm_goal_.route_roughing_area)
						arm_controller_.ready(arm_goal_.left_ready);
					arm_goal_.route = arm_goal_.route_rest;
					as_.setAborted(ArmResult(), "Arm finish tasks");
					ROS_WARN("Failed to operate ellipse.");
				}
				else
				{
					pose_publisher_.publish(msg);
				}
			}
		}
		else if (!debug_)
		{
			if (!arm_controller_.theta_turn)
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
						arm_controller_.ready_after_putting();
						arm_controller_.catch_after_putting(which_color(), false);
						next_task();
						arm_controller_.catch_after_putting(which_color(), false);
						next_task();
						arm_controller_.catch_after_putting(which_color(), true);
						next_task();
					}
					as_.setSucceeded(ArmResult(), "Arm finish tasks");
					if (arm_goal_.route == arm_goal_.route_roughing_area)
						arm_controller_.ready(arm_goal_.left_ready);
					finish_ = true;
					arm_goal_.route = arm_goal_.route_rest;
					ROS_INFO("Finish operating ellipse...");
				}
				// else if (!msg.end)
				// {
				// 	finish_adjusting_ = false;
				// }
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

	bool ArmServer::operate_border(const sensor_msgs::ImageConstPtr &image_rect,
								   sensor_msgs::ImagePtr &debug_image)
	{
		static ros::Time err_time;
		Pose2DMightEnd msg;
		if (finish_)
		{
			// static int cnt = 0;
			// if (arm_goal_.left_ready)
			// 	cnt++;
			// if (!arm_goal_.left_ready || (cnt >= 2))
			// {
			// static int cnt = 0;
			// cnt++;
			// if (cnt > 1) // 前方是停车区不调位姿
			// {
			msg.end = false;
			msg.pose.x = msg.not_change;
			msg.pose.y = msg.not_change;
			msg.pose.theta = msg.not_change;
			msg.header = image_rect->header;
			msg.header.frame_id = "base_footprint";
			ArmResult result;
			result.pme = msg;
			as_.setSucceeded(ArmResult(), "Arm finish tasks");
			arm_controller_.ready(arm_goal_.left_ready);
			arm_goal_.route = arm_goal_.route_rest;
			return true;
			// }
		}
		return false;
		/*		finish_adjusting_ = false;
				finish_ = false;
				ROS_INFO("Start to operate border...");
				err_time = image_rect->header.stamp;
			}
			bool valid = true;
			if (!finish_adjusting_)
			{
				msg.end = false;
				valid = arm_controller_.find_border(image_rect, msg, debug_image);
				if (valid)
				{
					if (msg.pose.theta != msg.not_change)
					{
						if ((image_rect->header.stamp - err_time).toSec() >= 13) // 超时但找到线
						{
							msg.end = true;
							pose_publisher_.publish(msg);
							ArmResult result;
							result.pme = msg;
							as_.setAborted(result, "Arm finish tasks");
							// 结束该函数
							arm_controller_.find_border(image_rect, msg, debug_image);
							arm_controller_.ready(arm_goal_.left_ready);
							finish_adjusting_ = true;
							finish_ = true;
							arm_goal_.route = arm_goal_.route_rest;
							ROS_WARN("Failed to operate border.");
						}
						else
						{
							pose_publisher_.publish(msg);
							if (msg.end)
							{
								ROS_INFO("y:%lf theta:%lf", msg.pose.y, msg.pose.theta);
								finish_adjusting_ = true;
								finish_ = true;
								ArmResult result;
								result.pme = msg;
								as_.setSucceeded(result, "Arm finish tasks");
								arm_controller_.ready(arm_goal_.left_ready);
								arm_goal_.route = arm_goal_.route_rest;
								ROS_INFO("Finish operating border...");
							}
						}
					}
					else
					{
						ArmFeedback feedback;
						feedback.pme = msg;
						as_.publishFeedback(feedback);
						err_time = image_rect->header.stamp;
					}
				}
				else
				{
					msg.pose.x = msg.not_change;
					msg.pose.y = msg.not_change;
					msg.pose.theta = msg.not_change;
					msg.header = image_rect->header;
					msg.header.frame_id = "base_footprint";
					if ((image_rect->header.stamp - err_time).toSec() >= 5) // 超时但没找到线
					{
						msg.end = true;
						pose_publisher_.publish(msg);
						ArmResult result;
						result.pme = msg;
						as_.setAborted(ArmResult(), "Arm finish tasks");
						// 结束该函数
						arm_controller_.find_border(image_rect, msg, debug_image);
						arm_controller_.ready(arm_goal_.left_ready);
						finish_adjusting_ = true;
						finish_ = true;
						arm_goal_.route = arm_goal_.route_rest;
						ROS_WARN("Failed to operate border.");
					}
					else
					{
						// 发送此数据表示车辆即刻停止，重新寻找定位物体
						msg.end = false;
						pose_publisher_.publish(msg);
						finish_adjusting_ = false;
					}
				}
			}
			return valid;*/
	}

	bool ArmServer::operate_parking_area(const sensor_msgs::ImageConstPtr &image_rect,
										 sensor_msgs::ImagePtr &debug_image)
	{
		static ros::Time err_time;
		if (finish_)
		{
			finish_adjusting_ = false;
			finish_ = false;
			ROS_INFO("Start to operate parking area...");
			err_time = image_rect->header.stamp;
		}
		bool valid = true;
		Pose2DMightEnd msg;
		if (!finish_adjusting_)
		{
			msg.end = false;
			valid = arm_controller_.find_parking_area(image_rect, msg, debug_image);
			if (valid)
			{
				// pose_publisher_.publish(msg);
				ArmFeedback feedback;
				feedback.pme = msg;
				as_.publishFeedback(feedback);
				err_time = image_rect->header.stamp;
				if (msg.end)
				{
					ROS_INFO("x:%lf y:%lf theta:%lf", msg.pose.x, msg.pose.y, msg.pose.theta);
					finish_adjusting_ = true;
					finish_ = true;
					ArmResult result;
					if (!debug_)
					{
						result.pme = msg;
					}
					else
						result.pme.end = false;
					as_.setSucceeded(result, "Arm finish tasks");
					arm_goal_.route = arm_goal_.route_rest;
					ROS_INFO("Finish operating parking area...");
				}
			}
			else
			{
				// 发送此数据表示车辆即刻停止，重新寻找定位物体
				msg.pose.x = msg.not_change;
				msg.pose.y = msg.not_change;
				msg.pose.theta = msg.not_change;
				msg.header = image_rect->header;
				msg.header.frame_id = "base_footprint";
				if ((image_rect->header.stamp - err_time).toSec() > 6) // 超时
				{
					finish_adjusting_ = true;
					finish_ = true;
					msg.end = true;
					ArmFeedback feedback;
					feedback.pme = msg;
					as_.publishFeedback(feedback);
					ros::Duration(0.2).sleep();
					arm_controller_.find_parking_area(image_rect, msg, debug_image);
					arm_goal_.route = arm_goal_.route_rest;
					ArmResult result;
					if (!debug_)
					{
						result.pme = msg;
					}
					else
						result.pme.end = false;
					// 结束该函数
					as_.setAborted(result, "Arm finish tasks");
					ROS_WARN("Failed to operating parking area.");
				}
				else
				{
					ArmFeedback feedback;
					feedback.pme = msg;
					as_.publishFeedback(feedback);
				}
			}
		}
		return valid;
	}

	void ArmServer::dr_callback(drConfig &config, uint32_t level)
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
		if (arm_controller_.target_pose.pose[arm_controller_.target_pose.target].x != a.x &&
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].x != Pose2DMightEnd::not_change)
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].x = a.x;
		if (arm_controller_.target_pose.pose[arm_controller_.target_pose.target].y != a.y &&
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].y != Pose2DMightEnd::not_change)
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].y = a.y;
		if (arm_controller_.target_pose.pose[arm_controller_.target_pose.target].theta !=
				Angle(config.target_theta_deg).rad() &&
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].theta !=
				Pose2DMightEnd::not_change)
			arm_controller_.target_pose.pose[arm_controller_.target_pose.target].theta =
				Angle(config.target_theta_deg).rad();
	}

	bool ArmServer::done_callback(moveDone::Request &req, moveDone::Response &resp)
	{
		ROS_INFO("Adjust angle done!");
		arm_controller_.theta_turn = req.theta_turn;
		if (!arm_controller_.theta_turn) // 0
		{
			arm_controller_.theta_turn = Pose2DMightEnd::not_change;
		}
		return true;
	}
} // namespace my_hand_eye
