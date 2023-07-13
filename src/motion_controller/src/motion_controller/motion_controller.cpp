#include "motion_controller/motion_controller.h"

namespace motion_controller
{
    MotionController::MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : delta_x_(0), delta_y_(0), delta_theta_(0),
          finish_turning_(false),
          listener_(buffer_), follower_(nh, pnh),
          ac_move_(nh, "Move", true), ac_arm_(nh, "Arm", true)
    {
        timer_ = nh.createTimer(ros::Rate(4), &MotionController::_timer_callback, this, false, false);
        if (!follower_.param_modification)
        {
            nh.setParam("/width_road", width_road_);
            vision_publisher = nh.advertise<Distance>("/vision_usb_cam", 5);
            go_client_ = nh.advertiseService("Go", &MotionController::go, this);
        }
        else
            timer_.start();
    }

    bool MotionController::_turn()
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        // 矫正pid结束位置与转弯位置的8cm误差
        motion_controller::MoveGoal goal1;
        goal1.pose.x = 0.08;
        ac_move_.sendGoalAndWait(goal1, ros::Duration(8), ros::Duration(0.1));
        motion_controller::MoveGoal goal2;
        if (get_position())
        {
            goal2.pose.theta = angle_corner();
            if (goal2.pose.theta == 0)
            {
                // 未到达弯道，重启弯道订阅者
                return false;
            }
        }
        else
            goal2.pose.theta = left_ ? M_PI / 2 : -M_PI / 2;
        // 转弯后前进一段距离
        goal2.pose.y = left_ ? 0.1 : -0.1;
        ac_move_.sendGoalAndWait(goal2, ros::Duration(13), ros::Duration(0.1));
        // 转弯后定时器将订阅关闭
        finish_turning_ = true;
        return true;
    }

    void MotionController::_U_turn()
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        motion_controller::MoveGoal goal;
        goal.pose.theta = angle_U_turn();
        if (!goal.pose.theta)
            goal.pose.theta = M_PI;
        ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
        // 需要改变之后的转弯方向
        left_ = !left_;
    }

    bool MotionController::_turn_by_position()
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        if (get_position())
        {
            // 矫正误差
            motion_controller::MoveGoal goal1;
            goal1.pose.x = length_corner();
            if (goal1.pose.x)
            {
                ac_move_.sendGoalAndWait(goal1, ros::Duration(15), ros::Duration(0.1));
                ROS_WARN("Turn by position ...");
            }
            else
                return false;
        }
        motion_controller::MoveGoal goal2;
        if (get_position())
        {
            goal2.pose.theta = angle_corner();
            if (goal2.pose.theta == 0)
            {
                // 未到达弯道，重启弯道订阅者
                return false;
            }
        }
        else
            goal2.pose.theta = left_ ? M_PI / 2 : -M_PI / 2;
        // 转弯后前进一段距离
        goal2.pose.y = left_ ? 0.1 : -0.1;
        ac_move_.sendGoalAndWait(goal2, ros::Duration(15), ros::Duration(0.1));
        // 转弯后定时器将订阅关闭
        finish_turning_ = true;
        return true;
    }

    void MotionController::_timer_callback(const ros::TimerEvent &event)
    {
        if (get_position())
        {
            if (follower_.param_modification)
            {
                if (!follower_.has_started && follower_.motor_status)
                {
                    set_position(0, 0, 0);
                    follower_.start(true, theta_);
                }
                follower_.follow(theta_, event.current_real);
            }
            else
            {
                // 转弯结束关闭转弯订阅
                if (finish_turning_)
                {
                    if (!can_turn())
                        follower_.start(false);
                    finish_turning_ = false;
                }
                if (arrive())
                {
                    if (!ac_arm_.isServerConnected())
                        ac_arm_.waitForServer();
                    my_hand_eye::ArmGoal goal;
                    switch (where_is_car())
                    {
                    case route_QR_code_board:
                        goal.route = goal.route_QR_code_board;
                        break;

                    case route_raw_material_area:
                        goal.route = goal.route_raw_material_area;
                        break;

                    case route_roughing_area:
                        goal.route = goal.route_roughing_area;
                        break;

                    case route_semi_finishing_area:
                        goal.route = goal.route_semi_finishing_area;
                        break;

                    case route_parking_area:
                        goal.route = goal.route_parking_area;
                        break;

                    default:
                        return;
                    }
                    ac_arm_.sendGoal(goal, boost::bind(&MotionController::_done_callback, this, _1, _2),
                                     boost::bind(&MotionController::_active_callback, this),
                                     boost::bind(&MotionController::_feedback_callback, this));
                    doing();
                }
            }
        }
    }

    void MotionController::_active_callback()
    {
        if (timer_.hasStarted())
            timer_.stop();
        if (where_is_car() == route_raw_material_area && loop_ == 1)
            _U_turn();
    }

    void MotionController::_feedback_callback() {}

    void MotionController::_done_callback(const actionlib::SimpleClientGoalState &state, const my_hand_eye::ArmResultConstPtr &result)
    {
        if (where_is_car() == route_raw_material_area || where_is_car() == route_roughing_area)
            follower_.start(true);
        else if (where_is_car() == route_semi_finishing_area)
        {
            if (loop_ == 0)
                _U_turn();
            follower_.start(true);
        }
        else if (where_is_car() == route_parking_area)
        {
            if (!ac_move_.isServerConnected())
                ac_move_.waitForServer();
            // 先移动到停车区的下侧
            get_position();
            motion_controller::MoveGoal goal1;
            goal1.pose.x = y_ - (y_road_up_up_ + length_car_ / 2);
            goal1.pose.theta = M_PI / 2;
            goal1.pose.y = -(x_ - length_car_ / 2);
            ac_move_.sendGoalAndWait(goal1, ros::Duration(20), ros::Duration(0.1));
            motion_controller::MoveGoal goal2;
            goal2.pose.y = -(y_road_up_up_ + length_car_ / 2 - length_parking_area_ / 2);
            ac_move_.sendGoalAndWait(goal2, ros::Duration(10), ros::Duration(0.1));
            if (get_position())
                ROS_INFO_STREAM("Finish! x: " << x_ << " y: " << y_);
            return;
        }

        finish();
        if (!timer_.hasStarted())
            timer_.start();
        follower_.start(true);
    }

    bool MotionController::set_position(double x, double y, double theta)
    {
        x_ = x;
        y_ = y;
        // 将theta_限定在(-pi,pi]之间
        theta_ = (theta <= -M_PI) ? theta + M_PI : (theta > M_PI ? theta - M_PI : theta);
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            delta_x_ = x - tfs.transform.translation.x;
            delta_y_ = y - tfs.transform.translation.y;
            // w = cos(theta/2) x = 0 y = 0 z = sin(theta/2)
            delta_theta_ = theta_ - atan2(tfs.transform.rotation.z, tfs.transform.rotation.w) * 2;
        }
        catch (const std::exception &e)
        {
            ROS_WARN("set_position exception:%s", e.what());
            return false;
        }
        return true;
    }

    bool MotionController::get_position()
    {
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            x_ = delta_x_ + tfs.transform.translation.x;
            y_ = delta_y_ + tfs.transform.translation.y;
            theta_ = delta_theta_ + atan2(tfs.transform.rotation.z, tfs.transform.rotation.w) * 2;
            // 将theta_限定在(-pi,pi]之间
            theta_ = (theta_ <= -M_PI) ? theta_ + M_PI : (theta_ > M_PI ? theta_ - M_PI : theta_);
        }
        catch (const std::exception &e)
        {
            ROS_WARN("get_position exception:%s", e.what());
            return false;
        }
        return true;
    }

    bool MotionController::go(Go::Request &req, Go::Response &resp)
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        if (!set_position(length_car_ / 2, length_car_ / 2, 0))
        {
            ROS_ERROR("Failed to initialize position!");
            return false;
        }
        // 横向移动出停止区
        motion_controller::MoveGoal goal1;
        goal1.pose.y = y_road_up_up_ + width_road_ / 2 - length_car_ / 2;
        ac_move_.sendGoalAndWait(goal1, ros::Duration(10), ros::Duration(0.1));
        // 直接前进到二维码板
        motion_controller::MoveGoal goal2;
        goal2.pose.x = x_QR_code_board_ - length_car_ / 2;
        ac_move_.sendGoalAndWait(goal2, ros::Duration(17), ros::Duration(0.1));
        timer_.start();
        return true;
    }
} // namespace motion_controller
