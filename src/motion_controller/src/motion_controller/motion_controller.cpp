#include <tf/tf.h>
#include <my_hand_eye/backward_kinematics.h>
#include "motion_controller/motion_controller.h"

namespace motion_controller
{
    MotionController::MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : delta_x_(0), delta_y_(0), delta_theta_(0),
          finish_turning_(true), // on_road_(false),
          listener_(buffer_), follower_(nh, pnh), dr_server_(follower_.mtx, pnh),
          ac_move_(nh, "Move", true), ac_arm_(nh, "Arm", true),
          move_active_(false), arm_active_(false),
          move_initialized_(false), arm_initialized_(false)
    {
        client_done_ = nh.serviceClient<my_hand_eye::moveDone>("moveDone");
        timer_ = nh.createTimer(ros::Rate(10), &MotionController::_timer_callback, this, false, false);
        timeout_ = pnh.param("timeout", 1.0);
        if (!follower_.debug)
        {
            server_go_ = nh.advertiseService("Go", &MotionController::go, this);
        }
        else
        {
            dr_server_.setCallback(boost::bind(&MotionController::_dr_callback, this, _1, _2));
            timer_.start();
        }
    }

    // bool MotionController::_turn()
    // {
    //     ac_move_.waitForServer();
    //     // 矫正pid结束位置与转弯位置的8cm误差
    //     motion_controller::MoveGoal goal1;
    //     goal1.pose.x = 0.08;
    //     ac_move_.sendGoalAndWait(goal1, ros::Duration(8), ros::Duration(0.1));
    //     motion_controller::MoveGoal goal2;
    //     if (get_position())
    //     {
    //         goal2.pose.theta = angle_corner();
    //         if (goal2.pose.theta == 0)
    //         {
    //             // 未到达弯道，重启弯道订阅者
    //             return false;
    //         }
    //     }
    //     else
    //         goal2.pose.theta = left_ ? M_PI / 2 : -M_PI / 2;
    //     // 转弯后前进一段距离
    //     goal2.pose.y = left_ ? 0.1 : -0.1;
    //     ac_move_.sendGoalAndWait(goal2, ros::Duration(13), ros::Duration(0.1));
    //     boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
    //     // 转弯后定时器将订阅关闭
    //     finish_turning_ = true;
    //     return true;
    // }

    // void MotionController::_U_turn()
    // {
    //     ac_move_.waitForServer();
    //     motion_controller::MoveGoal goal;
    //     goal.pose.theta = angle_U_turn();
    //     if (!goal.pose.theta)
    //         goal.pose.theta = M_PI;
    //     ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
    //     boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
    //     // 需要改变之后的转弯方向
    //     left_ = !left_;
    // }

    // bool MotionController::_turn_by_position()
    // {
    //     ac_move_.waitForServer();
    //     if (get_position())
    //     {
    //         // 矫正误差
    //         motion_controller::MoveGoal goal1;
    //         goal1.pose.x = length_corner();
    //         if (goal1.pose.x)
    //         {
    //             ac_move_.sendGoalAndWait(goal1, ros::Duration(15), ros::Duration(0.1));
    //             ROS_WARN("Turn by position ...");
    //         }
    //         else
    //             return false;
    //     }
    //     motion_controller::MoveGoal goal2;
    //     if (get_position())
    //     {
    //         goal2.pose.theta = angle_corner();
    //         if (goal2.pose.theta == 0)
    //         {
    //             // 未到达弯道，重启弯道订阅者
    //             return false;
    //         }
    //     }
    //     else
    //         goal2.pose.theta = left_ ? M_PI / 2 : -M_PI / 2;
    //     // 转弯后前进一段距离
    //     goal2.pose.y = left_ ? 0.1 : -0.1;
    //     ac_move_.sendGoalAndWait(goal2, ros::Duration(15), ros::Duration(0.1));
    //     boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
    //     // 转弯后定时器将订阅关闭
    //     finish_turning_ = true;
    //     return true;
    // }

    void MotionController::_timer_callback(const ros::TimerEvent &event)
    {
        if (follower_.debug)
        {
            static bool flag = true;
            if (flag && config_.startup)
            {
                ac_arm_.waitForServer();
                my_hand_eye::ArmGoal goal;
                goal.loop = loop_;
                if (where_is_car(follower_.debug, config_.startup) == route_rest)
                {
                    if (!follower_.has_started)
                    {
                        set_position(0, 0, -my_hand_eye::Angle(config_.theta_adjust).rad());
                        follower_.start(true, theta_, config_.dist,
                                        my_hand_eye::Angle(config_.theta_adjust).rad());
                    }
                    if (get_position())
                    {
                        ROS_INFO_STREAM("x:" << x_ << " y:" << y_ << " theta:" << theta_);
                        if (config_.theta_adjust)
                        {
                            bool ok = follower_.start_then_adjust(theta_,
                                                                  config_.dist - std::max(abs(x_), abs(y_)),
                                                                  event.current_real);
                            if (ok)
                            {
                                {
                                    boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                                    config_.theta_adjust = 0;
                                }
                                dr_server_.updateConfig(config_);
                            }
                        }
                        else
                            follower_.follow(theta_,
                                             config_.dist - std::max(abs(x_), abs(y_)), event.current_real);
                    }
                    return;
                }
                else if (where_is_car(follower_.debug, config_.startup) == route_QR_code_board &&
                         !follower_.has_started)
                {
                    set_position(-(x_road_up_ + width_road_ / 2 - length_car_ / 2), width_car_ / 2, 0);
                    follower_.start(true, theta_,
                                    abs(length_route(follower_.debug, config_.startup, 0)));
                }
                else if (where_is_car(follower_.debug, config_.startup) == route_roughing_area ||
                         where_is_car(follower_.debug, config_.startup) == route_semi_finishing_area)
                    set_position(
                        0, 0, (where_is_car(follower_.debug, config_.startup) == route_roughing_area) ? 0 : M_PI / 2);
                goal.route = where_is_car(follower_.debug, config_.startup);
                goal.left_ready = !(where_is_car(follower_.debug, config_.startup, 1) == route_raw_material_area ||
                                    where_is_car(follower_.debug, config_.startup, 1) == route_parking_area);
                ac_arm_.sendGoal(goal, boost::bind(&MotionController::_arm_done_callback, this, _1, _2),
                                 boost::bind(&MotionController::_arm_active_callback, this),
                                 boost::bind(&MotionController::_arm_feedback_callback, this, _1));
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                flag = false;
            }
            else if (!config_.startup && !flag)
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                flag = true; // 等待下一次启动
            }
            else if (where_is_car(follower_.debug, config_.startup) == route_raw_material_area && !flag)
            {
                _move_with_vision();
            }
            else if (where_is_car(follower_.debug, config_.startup) == route_QR_code_board && !flag)
            {
                if (get_position())
                {
                    if (arrived(follower_.debug, config_.startup) && follower_.has_started)
                    {
                        ROS_INFO("Shut down by timer.");
                        follower_.start(false, theta_);
                    }
                    if (follower_.has_started)
                    {
                        ROS_INFO_STREAM("x:" << x_ << " y:" << y_);
                        follower_.follow(theta_,
                                         abs(length_route(follower_.debug, config_.startup, 0)),
                                         event.current_real);
                    }
                }
            }
        }
        else if (get_position())
        {
            if (arrived(follower_.debug, config_.startup))
            {
                ac_arm_.waitForServer();
                my_hand_eye::ArmGoal goal;
                goal.loop = loop_;
                if (where_is_car(follower_.debug, config_.startup) == route_rest)
                {
                    ROS_ERROR("Assertion failed: where_is_car != route_rest");
                    return;
                }
                goal.route = where_is_car(follower_.debug, config_.startup);
                int last_route = where_is_car(follower_.debug, config_.startup, -1);
                int next_route = where_is_car(follower_.debug, config_.startup, 1);
                // 暂存任务点信息，防止其他进程更改
                goal.left_ready = !(next_route == route_raw_material_area ||
                                    next_route == route_parking_area);
                if (goal.route == route_QR_code_board)
                {
                    // 如果没停，且还没扫到码
                    if (follower_.has_started && ac_arm_.getState().toString() != "SUCCEEDED")
                    {
                        follower_.start(false, theta_);
                        doing(follower_.mtx);
                        {
                            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                            finish_turning_ = false;
                            // 用于防止follower提前打开或finished
                        }
                        ac_move_.waitForServer();
                        // 等车停
                        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
                        MoveGoal goal;
                        get_position();
                        goal.pose.theta = angle_from_road(follower_.debug, config_.startup);
                        goal.pose.y = length_route(follower_.debug, config_.startup) *
                                      cos(goal.pose.theta);
                        goal.pose.x = -length_route(follower_.debug, config_.startup) *
                                      sin(goal.pose.theta);
                        ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
                        {
                            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                            finish_turning_ = true;
                            // 用于防止follower提前打开或finished
                        }
                    }
                    return;
                }
                follower_.start(false, theta_);
                if (goal.route == route_roughing_area || goal.route == route_semi_finishing_area)
                {
                    // if (!follower_.start_then_adjust(theta_, event.current_real))
                    //     return;
                    ac_move_.waitForServer();
                    // 等车停
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
                    MoveGoal goal1;
                    get_position();
                    goal1.pose.theta = angle_from_road(follower_.debug, config_.startup);
                    ROS_INFO_STREAM("Move theta " << goal1.pose.theta);
                    double width = length_from_road(follower_.debug, config_.startup) -
                                   (goal.route == route_roughing_area
                                        ? width_from_roughing_area_
                                        : width_from_semi_finishing_area_) +
                                   width_road_ / 2;
                    goal1.pose.y = width * cos(goal1.pose.theta);
                    goal1.pose.x = -width * sin(goal1.pose.theta);
                    goal1.precision = true;
                    ac_move_.sendGoalAndWait(goal1, ros::Duration(15), ros::Duration(0.1));
                }
                else if (goal.route == route_raw_material_area)
                {
                    if (loop_ == 0)
                    {
                        ac_move_.waitForServer();
                        // 等车停
                        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
                        MoveGoal goal;
                        goal.pose.theta = angle_raw_material_area_ +
                                          angle_from_road(follower_.debug, config_.startup);
                        get_position();
                        goal.pose.y = length_route(follower_.debug, config_.startup);
                        ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
                    }
                    else if (loop_ == 1)
                    {
                        ac_move_.waitForServer();
                        // 等车停
                        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
                        get_position();
                        MoveGoal goal1;
                        goal1.pose.y = length_from_road(follower_.debug, config_.startup);
                        ac_move_.sendGoalAndWait(goal1, ros::Duration(15), ros::Duration(0.1));
                        // get_position();
                        // MoveGoal goal2;
                        // goal2.pose.x = length_border();
                        // ROS_INFO_STREAM("Move " << goal2.pose.x);
                        // ac_move_.sendGoalAndWait(goal2, ros::Duration(15), ros::Duration(0.1));
                        get_position();
                        MoveGoal goal3;
                        goal3.pose.theta = -angle_raw_material_area_ +
                                           angle_from_road(follower_.debug, config_.startup);
                        goal3.pose.y =
                            y_raw_material_area_ - y_ +
                            (radius_raw_material_area_ + width_road_ / 2) * tan(angle_raw_material_area_);
                        ac_move_.sendGoalAndWait(goal3, ros::Duration(15), ros::Duration(0.1));
                    }
                    else
                    {
                        ROS_ERROR("Invalid loop!");
                        return;
                    }
                }
                else if (goal.route == route_border &&
                         (last_route == route_roughing_area ||
                          last_route == route_semi_finishing_area))
                {
                    ac_move_.waitForServer();
                    // 等车停
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
                    MoveGoal goal;
                    int sign =
                        (last_route == route_semi_finishing_area &&
                         next_route == route_raw_material_area)
                            ? 1
                            : -1;
                    get_position();
                    goal.pose.x = sign * length_route(follower_.debug, config_.startup) *
                                  (clockwise_ ? -1 : 1);
                    ROS_INFO_STREAM("Move " << goal.pose.x * sign);
                    ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
                    // if (where_is_car(follower_.debug, config_.startup, 1) == route_parking_area)
                    // {
                    //     MoveGoal goal;
                    //     get_position();
                    //     goal.pose.theta = angle_corner();
                    //     ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
                    //     get_position();
                    //     follower_.veer(true, true);
                    //     follower_.start(true, theta_);
                    //     boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                    //     doing();
                    //     finished();
                    //     return;
                    // }
                }
                // 保证doing在finished之前
                {
                    doing(follower_.mtx);
                }
                ac_arm_.sendGoal(goal, boost::bind(&MotionController::_arm_done_callback, this, _1, _2),
                                 boost::bind(&MotionController::_arm_active_callback, this),
                                 boost::bind(&MotionController::_arm_feedback_callback, this, _1));
                if (goal.route == route_border)
                {
                    {
                        boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                        finish_turning_ = false;
                        // 用于防止server移动或follower提前打开
                    }
                    if (last_route == route_roughing_area ||
                        last_route == route_semi_finishing_area)
                    {
                        MoveGoal goal;
                        get_position();
                        goal.pose.theta = angle_corner();
                        // if (where_is_car(follower_.debug, config_.startup, 1) == route_parking_area)
                        // {
                        //     goal.pose.y = x_road_up_ + width_field_ - width_road_ -
                        //                   length_car_ / 2 - (-x_);
                        // }
                        ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
                        // if (where_is_car(follower_.debug, config_.startup, -1) == route_semi_finishing_area)
                        // {
                        //     get_position();
                        //     follower_.start(true, theta_);
                        // }
                    }
                    else
                    {
                        ac_move_.waitForServer();
                        // 等车停
                        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
                        MoveGoal goal;
                        get_position();
                        goal.pose.theta = angle_from_road(follower_.debug, config_.startup);
                        goal.pose.y = length_route(follower_.debug, config_.startup) *
                                      (clockwise_ ? -1 : 1) * cos(goal.pose.theta);
                        goal.pose.x = -length_route(follower_.debug, config_.startup) *
                                      (clockwise_ ? -1 : 1) * sin(goal.pose.theta);
                        ROS_INFO_STREAM("Move " << goal.pose.y);
                        ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
                    }
                    boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                    finish_turning_ = true;
                }
            }
            else if (where_is_car(follower_.debug, config_.startup) == route_raw_material_area &&
                     is_doing())
            {
                _move_with_vision();
            }
            else if (!is_doing())
            {
                // double dist = on_road_
                //                   ? abs(length_route(follower_.debug, config_.startup, 0))
                //                   : abs(length_from_road(follower_.debug, config_.startup, 0));
                follower_.follow(theta_, abs(length_route(follower_.debug, config_.startup, 0)), event.current_real);
                // if (ok && !on_road_)
                // {
                //     follower_.start(false, theta_);
                //     if (where_is_car(follower_.debug, config_.startup) == route_QR_code_board)
                //     {
                //         follower_.veer(false, true);
                //     }
                //     else
                //         ROS_ERROR("Car is not on the road. route: %d",
                //                   where_is_car(follower_.debug, config_.startup));
                //     follower_.start(true, theta_, abs(length_route(follower_.debug, config_.startup, 0)));
                //     boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                //     on_road_ = true;
                // }
                // else if (ok)
                // {
                //     ROS_WARN("Method 'follow' returns true but 'arrived' returns false.");
                // }
            }
        }
    }

    void MotionController::_check_arm_active()
    {
        if (!arm_stamp_.is_zero() && arm_pose_.theta == 0 &&
            arm_pose_.x == 0 && arm_pose_.y == 0 && finish_turning_)
        {
            ac_move_.waitForServer();
            ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
        }
        else if (!arm_active_ && !arm_stamp_.is_zero() && finish_turning_) // is_zero说明出现问题
        {
            {
                if (!arm_initialized_)
                {
                    MoveGoal goal;
                    goal.pose = arm_pose_;
                    ROS_INFO_STREAM("First move... x:" << goal.pose.x << " y:" << goal.pose.y << " theta:" << goal.pose.theta);
                    ac_move_.waitForServer();
                    ac_move_.sendGoal(goal, boost::bind(&MotionController::_move_done_callback, this, _1, _2),
                                      boost::bind(&MotionController::_move_active_callback, this),
                                      boost::bind(&MotionController::_move_feedback_callback, this, _1));
                    boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                    arm_initialized_ = true;
                }
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                arm_active_ = true;
            }
        }
    }

    void MotionController::_check_arm_pose(const my_hand_eye::Pose2DMightEnd &pme)
    {
        boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
        arm_pose_ = pme.pose;
        if (pme.pose.theta == pme.not_change)
            arm_pose_.theta = 0;
        if (pme.pose.x == pme.not_change)
            arm_pose_.x = 0;
        if (pme.pose.y == pme.not_change)
            arm_pose_.y = 0;
    }

    void MotionController::_arm_active_callback()
    {
        if (timer_.hasStarted() &&
            where_is_car(follower_.debug, config_.startup) != route_raw_material_area &&
            where_is_car(follower_.debug, config_.startup != route_QR_code_board) &&
            (where_is_car(follower_.debug, config_.startup) != route_border ||
             where_is_car(follower_.debug, config_.startup, 1) != route_parking_area))
            timer_.stop();
    }

    void MotionController::_arm_feedback_callback(const my_hand_eye::ArmFeedbackConstPtr &feedback)
    {
        if (where_is_car(follower_.debug, config_.startup) == route_raw_material_area)
        {
            if (feedback->pme.end)
            {
                if (timer_.hasStarted())
                    timer_.stop();
                ROS_INFO("Succeeded to move with vision");
                ac_move_.waitForServer();
                ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                arm_initialized_ = arm_active_ = false;
                move_initialized_ = move_active_ = false;
                return;
            }
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                arm_stamp_ = feedback->pme.header.stamp;
                arm_time_ = ros::Time::now();
            }
            _check_arm_pose(feedback->pme);
            _check_arm_active();
        }
        else if (where_is_car(follower_.debug, config_.startup) == route_roughing_area ||
                 where_is_car(follower_.debug, config_.startup) == route_semi_finishing_area)
        {
            if (feedback->pme.end)
            {
                if (feedback->pme.pose.theta != feedback->pme.not_change)
                {
                    get_position();
                    // double theta = angle_correction(feedback->pme.pose.theta);
                    // theta = (theta + theta_) / 2;
                    ROS_INFO("Before setting: x: %lf y:%lf theta:%lf", x_, y_, theta_);
                    if (where_is_car(follower_.debug, config_.startup) == route_roughing_area)
                    {
                        set_position(-(x_roughing_area_ + length_from_ellipse_ +
                                       feedback->pme.pose.x * cos(feedback->pme.pose.theta) +
                                       feedback->pme.pose.y * sin(feedback->pme.pose.theta)),
                                     length_field_ - width_from_roughing_area_ -
                                         feedback->pme.pose.y * cos(feedback->pme.pose.theta) -
                                         feedback->pme.pose.x * sin(feedback->pme.pose.theta),
                                     theta_);
                    }
                    else if (loop_ == 1)
                    {
                        set_position(x_,
                                     y_semi_finishing_area_ - length_from_ellipse_ -
                                         feedback->pme.pose.x * cos(feedback->pme.pose.theta) -
                                         feedback->pme.pose.y * sin(feedback->pme.pose.theta),
                                     theta_);
                    }
                    else if (loop_ == 0)
                    {
                        set_position(-(x_road_up_ + width_field_ - width_from_semi_finishing_area_ -
                                       feedback->pme.pose.y * cos(feedback->pme.pose.theta) -
                                       feedback->pme.pose.x * sin(feedback->pme.pose.theta)),
                                     y_semi_finishing_area_ - length_from_ellipse_ -
                                         feedback->pme.pose.x * cos(feedback->pme.pose.theta) -
                                         feedback->pme.pose.y * sin(feedback->pme.pose.theta),
                                     theta_);
                    }
                    else
                        ROS_ERROR("Invalid loop!");
                    ROS_INFO("After setting: x: %lf y:%lf theta:%lf", x_, y_, theta_);
                }
                ac_move_.waitForServer();
                MoveGoal goal;
                get_position();
                goal.pose.theta = angle_from_road(follower_.debug, config_.startup);
                goal.precision = true;
                ac_move_.sendGoalAndWait(goal, ros::Duration(8), ros::Duration(0.1));
                if (client_done_.exists())
                {
                    my_hand_eye::moveDone md;
                    get_position();
                    md.request.theta_turn = angle_from_road(follower_.debug, config_.startup);
                    if (!client_done_.call(md))
                        ROS_WARN("Failed to call moveDone!");
                }
            }
        }
        else if (where_is_car(follower_.debug, config_.startup) == route_border)
        {
            if (!feedback->pme.end)
            {
                {
                    boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                    arm_stamp_ = feedback->pme.header.stamp;
                }
                _check_arm_pose(feedback->pme);
                _check_arm_active();
                // 里程计/视觉分离：对传感器时间差进行比较
                double diff = 0;
                if (move_active_ && arm_active_ && !arm_stamp_.is_zero() && !move_stamp_.is_zero())
                {
                    diff = ros::Duration(arm_stamp_ - move_stamp_).toSec();
                    if (diff > 0.5)
                        ROS_INFO("Timestamps of arm and move are %f seconds apart.", diff);
                }
                ros::Time now = ros::Time::now();
                // ROS_INFO_STREAM((now - move_time_).toSec());
                if (move_active_ && ((now - move_time_).toSec() > timeout_ || move_stamp_.is_zero()))
                {
                    {
                        boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                        move_active_ = false;
                    }
                    if (move_stamp_.is_zero())
                        ROS_WARN("Move feedback not active any more");
                    else
                        ROS_WARN("Move feedback not active any more, timeout:%lf", (now - move_stamp_).toSec());
                }
                // ROS_INFO_STREAM(move_initialized_ << " " << arm_initialized_ << " " << arm_active_ << " " << diff << " " << move_active_);
                if (move_initialized_ && arm_initialized_ && arm_active_ &&
                    (diff > 0.5 || !move_active_))
                {
                    MoveGoal goal;
                    goal.pose = arm_pose_;
                    ROS_INFO_STREAM("x:" << goal.pose.x << " y:" << goal.pose.y << " theta:" << goal.pose.theta);
                    ac_move_.waitForServer();
                    ac_move_.sendGoal(goal, boost::bind(&MotionController::_move_done_callback, this, _1, _2),
                                      boost::bind(&MotionController::_move_active_callback, this),
                                      boost::bind(&MotionController::_move_feedback_callback, this, _1));
                }
            }
        }
    }

    void MotionController::_arm_done_callback(const actionlib::SimpleClientGoalState &state,
                                              const my_hand_eye::ArmResultConstPtr &result)
    {
        if (where_is_car(follower_.debug, config_.startup) == route_QR_code_board)
        {
            if (follower_.debug)
            {
                if (follower_.has_started)
                {
                    follower_.start(false, theta_);
                    ROS_INFO("Shut down by done cb.");
                }
            }
            // else
            // {
            //     if (follower_.has_started)
            //         follower_.start(false, theta_);
            //     follower_.veer(true, true);
            // }
            else
            {
                if (follower_.has_started)
                    follower_.start(false, theta_);
                ac_move_.waitForServer();
                MoveGoal goal;
                get_position();
                goal.pose.x = length_from_road(follower_.debug, config_.startup, 1);
                ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
            }
        }
        else if (where_is_car(follower_.debug, config_.startup) == route_raw_material_area &&
                 !follower_.debug)
        {
            MoveGoal goal;
            ac_move_.waitForServer();
            get_position();
            goal.pose.theta = angle_from_road(follower_.debug, config_.startup);
            // goal.pose.x = length_from_road(follower_.debug, config_.startup) * cos(-goal.pose.theta);
            // goal.pose.y = -length_from_road(follower_.debug, config_.startup) * sin(-goal.pose.theta);
            ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
            if (loop_ == 1)
            {
                // get_position();
                // MoveGoal goal;
                // goal.pose.y = length_border();
                // ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
                follower_.veer(false, true);
            }
        }
        else if ((where_is_car(follower_.debug, config_.startup) == route_roughing_area ||
                  where_is_car(follower_.debug, config_.startup) == route_semi_finishing_area) &&
                 !follower_.debug)
        {
            // follower_.veer(false, true);
            // ac_move_.waitForServer();
            // MoveGoal goal;
            // get_position();
            // goal.pose.y = length_from_road(follower_.debug, config_.startup);
            // ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
            if (where_is_car(follower_.debug, config_.startup) == route_semi_finishing_area && loop_ == 0)
                follower_.veer(true, true);
        }
        else if (where_is_car(follower_.debug, config_.startup) == route_border && !follower_.debug)
        {
            if (!follower_.debug && result->pme.pose.y != result->pme.not_change &&
                result->pme.pose.theta != result->pme.not_change &&
                result->pme.end) // 如果下一步是启停区或原料区不进
            {
                ac_move_.waitForServer();
                ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
                get_position();
                double theta, x, y;
                if (position_in_corner(result->pme.pose.y, result->pme.pose.theta,
                                       x, y, theta, !(y_ < width_road_)))
                    set_position(x, y, theta); // 进一步可以引入时间提高精确度
            }
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                arm_initialized_ = arm_active_ = false;
                move_initialized_ = move_active_ = false;
            }
            if (where_is_car(follower_.debug, config_.startup, 1) == route_roughing_area)
                follower_.veer(true, false);
            else if (where_is_car(follower_.debug, config_.startup, 1) == route_parking_area)
                follower_.veer(true, true);
        }
        else if (where_is_car(follower_.debug, config_.startup) == route_parking_area)
        {
            if (result->pme.end)
            {
                _check_arm_pose(result->pme);
                ac_move_.waitForServer();
                MoveGoal goal;
                goal.pose.x = length_from_parking_area_ * cos(arm_pose_.theta) +
                              arm_pose_.x;
                goal.pose.y = arm_pose_.y +
                              length_from_parking_area_ * sin(arm_pose_.theta);
                // goal.pose.theta = arm_pose_.theta;
                goal.precision = true;
                ac_move_.sendGoalAndWait(goal, ros::Duration(20), ros::Duration(0.1));
                if (follower_.debug)
                {
                    {
                        boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                        config_.startup = false;
                    }
                    dr_server_.updateConfig(config_);
                    if (!timer_.hasStarted())
                        timer_.start();
                }
                else
                    ros::shutdown();
                return;
            }
        }
        // 利用里程计停车
        // else if (where_is_car(follower_.debug, config_.startup) == route_parking_area)
        // {
        //     ac_move_.waitForServer();
        //     // 先移动到停车区的下侧
        //     get_position();
        //     motion_controller::MoveGoal goal1;
        //     goal1.pose.theta = angle_from_road();
        //     goal1.pose.y = -(y_ - length_parking_area_ / 2);
        //     ac_move_.sendGoalAndWait(goal1, ros::Duration(20), ros::Duration(0.1));
        //     motion_controller::MoveGoal goal2;
        //     get_position();
        //     goal2.pose.x = -x_ - length_parking_area_ / 2;
        //     ac_move_.sendGoalAndWait(goal2, ros::Duration(10), ros::Duration(0.1));
        //     if (get_position())
        //         ROS_INFO_STREAM("Finish! x: " << (-x_ - length_parking_area_ / 2) << " y: " << (y_ - length_parking_area_ / 2));
        //     return;
        // }
        if (state.toString() == "SUCCEEDED")
            ROS_INFO_STREAM("*** Arm finished: " << state.toString());
        else
        {
            // if (where_is_car(follower_.debug, config_.startup) == route_border && !follower_.debug)
            // {
            //     ac_move_.waitForServer();
            //     MoveGoal goal;
            //     get_position();
            //     goal.pose.theta = angle_from_road(follower_.debug, config_.startup);
            //     ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
            // }
            ROS_WARN_STREAM("*** Arm finished: " << state.toString());
        }
        if (!follower_.debug)
        {
            while (!finish_turning_)
            {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
            }
            if (!follower_.has_started)
            {
                get_position();
                // if (where_is_car(follower_.debug, config_.startup) == route_QR_code_board)
                // {
                //     follower_.start(true, theta_,
                //                     abs(length_from_road(follower_.debug, config_.startup, 1)));
                //     boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                //     on_road_ = false;
                // }
                // else if (where_is_car(follower_.debug, config_.startup) == route_roughing_area ||
                //          where_is_car(follower_.debug, config_.startup) == route_semi_finishing_area)
                // {
                //     follower_.start(true, theta_,
                //                     abs(length_from_road(follower_.debug, config_.startup)));
                //     boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                //     on_road_ = false;
                // }
                // else
                follower_.start(true, theta_, abs(length_route(follower_.debug, config_.startup, 1)));
            }
            if (!timer_.hasStarted())
                timer_.start();
            finished(follower_.mtx);
        }
        else
        {
            // dr_server_.updateConfigInternal
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                config_.startup = false;
            }
            dr_server_.updateConfig(config_);
            if (!timer_.hasStarted())
                timer_.start();
        }
    }

    void MotionController::_move_active_callback(){};

    void MotionController::_move_feedback_callback(const motion_controller::MoveFeedbackConstPtr &feedback)
    {
        if (where_is_car(follower_.debug, config_.startup) == route_raw_material_area ||
            where_is_car(follower_.debug, config_.startup) == route_border)
        {
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                move_stamp_ = feedback->header.stamp;
                move_time_ = ros::Time::now();
                move_pose_ = feedback->pose_now;
            }
            if (!move_active_ && !move_stamp_.is_zero()) // is_zero说明坐标转换失败
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                if (!move_initialized_)
                {
                    move_initialized_ = true;
                }
                move_active_ = true;
            }
        }
    }

    void MotionController::_move_done_callback(const actionlib::SimpleClientGoalState &state,
                                               const motion_controller::MoveResultConstPtr &result)
    {
        if (state.toString() == "SUCCEEDED")
            ROS_INFO_STREAM("*** Move finished: " << state.toString());
        else
            ROS_WARN_STREAM("*** Move finished: " << state.toString());
    }

    bool MotionController::set_position(double x, double y, double theta)
    {
        {
            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
            x_ = x;
            y_ = y;
            // 将theta_限定在(-pi,pi]之间
            theta_ = (theta <= -M_PI) ? theta + 2 * M_PI : (theta > M_PI ? theta - 2 * M_PI : theta);
        }
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
            double yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(tfs.transform.rotation, quat);
            double roll, pitch;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            delta_theta_ = theta_ - yaw;
            delta_x_ = x_ - tfs.transform.translation.x * cos(delta_theta_) + tfs.transform.translation.y * sin(delta_theta_);
            delta_y_ = y_ - tfs.transform.translation.y * cos(delta_theta_) - tfs.transform.translation.x * sin(delta_theta_);
        }
        catch (const std::exception &e)
        {
            ROS_WARN("set_position exception:%s", e.what());
            return false;
        }
        return true;
    }

    void MotionController::_dr_callback(routeConfig &config, uint32_t level)
    {
        follower_.dr(config);
        if (config_.startup != config.startup)
        {
            if ((dr_route_ == route_rest || dr_route_ == route_QR_code_board) &&
                !config.startup && follower_.has_started)
            {
                follower_.start(false);
            }
            if (!config.startup)
            {
                if (dr_route_ != route_rest)
                    ac_arm_.cancelAllGoals();
                if (dr_route_ == route_raw_material_area && timer_.hasStarted())
                {
                    ac_move_.waitForServer();
                    ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
                    ac_move_.cancelAllGoals();
                    boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                    arm_initialized_ = arm_active_ = false;
                    move_initialized_ = move_active_ = false;
                }
                if (!timer_.hasStarted())
                    timer_.start();
            }
        }
        if (config.where != dr_route_)
        {
            if (config.startup)
            {
                ac_arm_.cancelAllGoals();
                if (dr_route_ == route_raw_material_area && timer_.hasStarted())
                {
                    ac_move_.waitForServer();
                    ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
                    ac_move_.cancelAllGoals();
                    boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                    arm_initialized_ = arm_active_ = false;
                    move_initialized_ = move_active_ = false;
                }
                ROS_WARN("Please shut down startup!");
            }
            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
            dr_route_ = config.where;
        }
        if (config.loop != loop_)
        {
            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
            loop_ = config.loop;
        }
        boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
        config_ = config;
    }

    void MotionController::_move_with_vision()
    {
        // 利用定时器spin
        double diff = 0;
        // 里程计/视觉分离：对传感器时间差进行比较
        if (move_active_ && arm_active_ && !arm_stamp_.is_zero() && !move_stamp_.is_zero())
        {
            diff = ros::Duration(arm_stamp_ - move_stamp_).toSec();
            if (diff > 2.0)
                ROS_ERROR("Timestamps of arm and move are %f seconds apart.", diff);
        }
        ros::Time now = ros::Time::now();
        // check which sensors are still active
        // 里程计/视觉活动：对接收时间进行比较 !is_zero()
        if (move_active_ && ((now - move_time_).toSec() > timeout_ || move_stamp_.is_zero()))
        {
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                move_active_ = false;
            }
            if (move_stamp_.is_zero())
                ROS_WARN("Move feedback not active any more");
            else
                ROS_WARN("Move feedback not active any more, timeout:%lf", (now - move_stamp_).toSec());
        }
        if (arm_active_ && ((now - arm_time_).toSec() > timeout_ || arm_stamp_.is_zero()))
        {
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                arm_active_ = false;
            }
            if (arm_stamp_.is_zero())
                ROS_WARN("Arm feedback not active any more");
            else
                ROS_WARN("Arm feedback not active any more, timeout:%lf", (now - arm_stamp_).toSec());
        }
        // 判定里程计偏离
        // 条件：均已初始化完毕，且视觉活动
        // 1.当里程计位置偏离视觉位置时，且里程计滞后于视觉时间
        // 2.当里程计不活动
        // 3.当里程计与视觉偏离，且视觉超前
        if (move_initialized_ && arm_initialized_ && arm_active_ &&
            (diff > 2.0 || !move_active_ ||
             (diff > 0.5 && (fabs(move_pose_.x - arm_pose_.x) > 0.05 ||
                             fabs(move_pose_.y - arm_pose_.y) > 0.05 ||
                             fabs(move_pose_.theta - arm_pose_.theta) > 0.1))))
        {
            if (diff > 0.5 && (fabs(move_pose_.x - arm_pose_.x) > 0.05 ||
                               fabs(move_pose_.y - arm_pose_.y) > 0.05 ||
                               fabs(move_pose_.theta - arm_pose_.theta) > 0.1))
                ROS_WARN("Pose of move is invalid!");
            {
                boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
                move_active_ = false;
                move_initialized_ = false;
            }
            ac_move_.waitForServer();
            MoveGoal goal;
            goal.pose = arm_pose_;
            ROS_INFO_STREAM("x:" << goal.pose.x << " y:" << goal.pose.y << " theta:" << goal.pose.theta);
            ac_move_.sendGoal(goal, boost::bind(&MotionController::_move_done_callback, this, _1, _2),
                              boost::bind(&MotionController::_move_active_callback, this),
                              boost::bind(&MotionController::_move_feedback_callback, this, _1));
        }
        // 视觉不活动，说明找不到目标对象
        if (move_initialized_ && arm_initialized_ && !arm_active_)
        {
            // 停下重新找
            ac_move_.waitForServer();
            ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
            arm_initialized_ = false;
            move_initialized_ = move_active_ = false;
        }
    }

    bool MotionController::get_position()
    {
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            boost::lock_guard<boost::recursive_mutex> lk(follower_.mtx);
            x_ = delta_x_ + tfs.transform.translation.x * cos(delta_theta_) -
                 tfs.transform.translation.y * sin(delta_theta_);
            y_ = delta_y_ + tfs.transform.translation.y * cos(delta_theta_) +
                 tfs.transform.translation.x * sin(delta_theta_);
            tf::Quaternion quat;
            tf::quaternionMsgToTF(tfs.transform.rotation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            theta_ = delta_theta_ + yaw;
            // theta_ = delta_theta_ + atan2(tfs.transform.rotation.z, tfs.transform.rotation.w) * 2;
            // 将theta_限定在(-pi,pi]之间
            theta_ = (theta_ <= -M_PI) ? theta_ + 2 * M_PI : (theta_ > M_PI ? theta_ - 2 * M_PI : theta_);
        }
        catch (const std::exception &e)
        {
            ROS_WARN("get_position exception:%s", e.what());
            return false;
        }
        return true;
    }

    bool MotionController::go(Go::Request &req, Go::Response &res)
    {
        if (!set_position(-length_car_ / 2, width_car_ / 2, 0))
        {
            ROS_ERROR("Failed to initialize position!");
            return false;
        }
        ac_move_.waitForServer();
        // 横向移动出停止区
        motion_controller::MoveGoal goal1;
        get_position();
        goal1.pose.x = length_from_road(follower_.debug, config_.startup);
        ac_move_.sendGoalAndWait(goal1, ros::Duration(5), ros::Duration(0.1));
        // 发送二维码请求
        ac_arm_.waitForServer();
        my_hand_eye::ArmGoal goal;
        goal.loop = loop_;
        goal.route = where_is_car(follower_.debug, config_.startup);
        ac_arm_.sendGoal(goal, boost::bind(&MotionController::_arm_done_callback, this, _1, _2),
                         boost::bind(&MotionController::_arm_active_callback, this),
                         boost::bind(&MotionController::_arm_feedback_callback, this, _1));
        // 横向移动出停止区
        // follower_.start(true, theta_, abs(length_from_road(follower_.debug, config_.startup)));
        get_position();
        follower_.start(true, theta_, abs(length_route(follower_.debug, config_.startup)));
        timer_.start();
        // 直接前进到二维码板
        // motion_controller::MoveGoal goal2;
        // goal2.pose.x = x_QR_code_board_ - length_car_ / 2;
        // ac_move_.sendGoalAndWait(goal2, ros::Duration(17), ros::Duration(0.1));
        return true;
    }
} // namespace motion_controller
