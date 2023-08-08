#include "motion_controller/motion_controller.h"

namespace motion_controller
{
    MotionController::MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : delta_x_(0), delta_y_(0), delta_theta_(0),
          finish_turning_(false),
          listener_(buffer_), follower_(nh, pnh),
          ac_move_(nh, "Move", true), ac_arm_(nh, "Arm", true),
          move_active_(false), arm_active_(false),
          move_initialized_(false), arm_initialized_(false), dr_route_(route_rest)
    {
        timer_ = nh.createTimer(ros::Rate(4), &MotionController::_timer_callback, this, false, false);
        timeout_ = pnh.param("timeout", 1.0);
        if (!follower_.debug)
        {
            go_client_ = nh.advertiseService("Go", &MotionController::go, this);
        }
        else
        {
            dr_server_.setCallback(boost::bind(&MotionController::_dr_callback, this, _1, _2));
            timer_.start();
        }
    }

    bool MotionController::_turn()
    {
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
        boost::lock_guard<boost::mutex> lk(mtx_);
        // 转弯后定时器将订阅关闭
        finish_turning_ = true;
        return true;
    }

    void MotionController::_U_turn()
    {
        ac_move_.waitForServer();
        motion_controller::MoveGoal goal;
        goal.pose.theta = angle_U_turn();
        if (!goal.pose.theta)
            goal.pose.theta = M_PI;
        ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
        boost::lock_guard<boost::mutex> lk(mtx_);
        // 需要改变之后的转弯方向
        left_ = !left_;
    }

    bool MotionController::_turn_by_position()
    {
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
        boost::lock_guard<boost::mutex> lk(mtx_);
        // 转弯后定时器将订阅关闭
        finish_turning_ = true;
        return true;
    }

    void MotionController::_timer_callback(const ros::TimerEvent &event)
    {
        if (follower_.debug)
        {
            static bool flag = true;
            if (flag && follower_.startup)
            {
                ac_arm_.waitForServer();
                my_hand_eye::ArmGoal goal;
                goal.loop = loop_;
                switch (dr_route_)
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

                case route_border:
                    goal.route = goal.route_border;
                    break;

                case route_rest:
                    if (get_position())
                    {
                        if (!follower_.has_started && follower_.startup)
                        {
                            set_position(0, 0, 0);
                            boost::lock_guard<boost::mutex> lk(mtx_);
                            follower_.start(true, theta_);
                        }
                        boost::lock_guard<boost::mutex> lk(mtx_);
                        follower_.follow(theta_, event.current_real);
                    }
                    return;

                default:
                    return;
                }
                ac_arm_.sendGoal(goal, boost::bind(&MotionController::_arm_done_callback, this, _1, _2),
                                 boost::bind(&MotionController::_arm_active_callback, this),
                                 boost::bind(&MotionController::_arm_feedback_callback, this, _1));
                flag = false;
            }
            else if (!follower_.startup && !flag)
            {
                flag = true; // 等待下一次启动
            }
            else if (dr_route_ == route_raw_material_area && follower_.startup && !flag)
            {
                _move_with_vision();
            }
        }
        else if (get_position())
        {
            // 转弯结束关闭转弯订阅
            // if (finish_turning_)
            // {
            //     boost::lock_guard<boost::mutex> lk(mtx_);
            //     if (!can_turn())
            //         follower_.start(false);
            //     finish_turning_ = false;
            // }
            if (arrive())
            {
                ac_arm_.waitForServer();
                my_hand_eye::ArmGoal goal;
                goal.loop = loop_;
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
                ac_arm_.sendGoal(goal, boost::bind(&MotionController::_arm_done_callback, this, _1, _2),
                                 boost::bind(&MotionController::_arm_active_callback, this),
                                 boost::bind(&MotionController::_arm_feedback_callback, this, _1));
                boost::lock_guard<boost::mutex> lk(mtx_);
                doing();
            }
        }
    }

    void MotionController::_arm_active_callback()
    {
        if (timer_.hasStarted() &&
            (((where_is_car() != route_raw_material_area) && !follower_.debug) ||
             (follower_.debug && follower_.startup && dr_route_ != route_raw_material_area)))
            timer_.stop();
        // if (where_is_car() == route_raw_material_area && loop_ == 1)
        //     _U_turn();
    }

    void MotionController::_arm_feedback_callback(const my_hand_eye::ArmFeedbackConstPtr &feedback)
    {
        if ((where_is_car() == route_raw_material_area && !follower_.debug) ||
            (follower_.debug && follower_.startup && dr_route_ == route_raw_material_area))
        {
            if (feedback->pme.end)
            {
                if (timer_.hasStarted())
                    timer_.stop();
                ROS_INFO("Succeeded to move with vision");
                ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
                boost::lock_guard<boost::mutex> lk(mtx_);
                arm_initialized_ = arm_active_ = false;
                move_initialized_ = move_active_ = false;
                return;
            }
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                arm_stamp_ = feedback->pme.header.stamp;
                arm_time_ = ros::Time::now();
                arm_pose_ = feedback->pme.pose;
                if (arm_pose_.theta == feedback->pme.not_change)
                    arm_pose_.theta = 0;
                if (arm_pose_.x == feedback->pme.not_change)
                    arm_pose_.x = 0;
                if (arm_pose_.y == feedback->pme.not_change)
                    arm_pose_.y = 0;
            }
            if (!arm_active_ && !arm_stamp_.is_zero()) // is_zero说明出现问题
            {
                if (!arm_initialized_)
                {
                    MoveGoal goal;
                    goal.pose = arm_pose_;
                    ROS_INFO_STREAM("First move... x:" << goal.pose.x << " y:" << goal.pose.y << " theta:" << goal.pose.theta);
                    ac_move_.sendGoal(goal, boost::bind(&MotionController::_move_done_callback, this, _1, _2),
                                      boost::bind(&MotionController::_move_active_callback, this),
                                      boost::bind(&MotionController::_move_feedback_callback, this, _1));
                    boost::lock_guard<boost::mutex> lk(mtx_);
                    arm_initialized_ = true;
                }
                boost::lock_guard<boost::mutex> lk(mtx_);
                arm_active_ = true;
            }
            else if (!arm_stamp_.is_zero() && arm_pose_.theta == 0 && arm_pose_.x == 0 && arm_pose_.y == 0)
            {
                ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
            }
        }
    }

    void MotionController::_arm_done_callback(const actionlib::SimpleClientGoalState &state,
                                              const my_hand_eye::ArmResultConstPtr &result)
    {
        if (state.toString() == "SUCCEEDED")
            ROS_INFO_STREAM("*** Arm finished: " << state.toString());
        else
            ROS_ERROR_STREAM("*** Arm finished: " << state.toString());

        // if (where_is_car() == route_raw_material_area || where_is_car() == route_roughing_area)
        //     follower_.start(true);
        // else if (where_is_car() == route_semi_finishing_area)
        // {
        //     if (loop_ == 0)
        //         _U_turn();
        //     follower_.start(true);
        // }
        // else if (where_is_car() == route_parking_area)
        // {
        //     ac_move_.waitForServer();
        //     // 先移动到停车区的下侧
        //     get_position();
        //     motion_controller::MoveGoal goal1;
        //     goal1.pose.x = y_ - (y_road_up_up_ + length_car_ / 2);
        //     goal1.pose.theta = M_PI / 2;
        //     goal1.pose.y = -(x_ - length_car_ / 2);
        //     ac_move_.sendGoalAndWait(goal1, ros::Duration(20), ros::Duration(0.1));
        //     motion_controller::MoveGoal goal2;
        //     goal2.pose.y = -(y_road_up_up_ + length_car_ / 2 - length_parking_area_ / 2);
        //     ac_move_.sendGoalAndWait(goal2, ros::Duration(10), ros::Duration(0.1));
        //     if (get_position())
        //         ROS_INFO_STREAM("Finish! x: " << x_ << " y: " << y_);
        //     return;
        // }

        if (!follower_.debug)
        {
            boost::lock_guard<boost::mutex> lk(mtx_);
            finish();
            // follower_.start(true);
            if (!timer_.hasStarted())
                timer_.start();
        }
    }

    void MotionController::_move_active_callback(){};

    void MotionController::_move_feedback_callback(const motion_controller::MoveFeedbackConstPtr &feedback)
    {
        if ((where_is_car() == route_raw_material_area && !follower_.debug) ||
            (follower_.debug && follower_.startup && dr_route_ == route_raw_material_area))
        {
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                move_stamp_ = feedback->header.stamp;
                move_time_ = ros::Time::now();
                move_pose_ = feedback->pose_now;
            }
            if (!move_active_ && !move_stamp_.is_zero()) // is_zero说明坐标转换失败
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
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
            boost::lock_guard<boost::mutex> lk(mtx_);
            x_ = x;
            y_ = y;
            // 将theta_限定在(-pi,pi]之间
            theta_ = (theta <= -M_PI) ? theta + M_PI : (theta > M_PI ? theta - M_PI : theta);
        }
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            boost::lock_guard<boost::mutex> lk(mtx_);
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

    void MotionController::_dr_callback(routeConfig &config, uint32_t level)
    {
        // boost::lock_guard对象构造时，自动调用mtx.lock()进行上锁
        // boost::lock_guard对象析构时，自动调用mtx.unlock()释放锁
        {
            boost::lock_guard<boost::mutex> lk(mtx_);
            follower_.dr(config);
        }
        if (follower_.startup != config.startup)
        {
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                follower_.startup = config.startup;
            }
            if (dr_route_ == route_rest && !follower_.startup && follower_.has_started)
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                follower_.start(false);
            }
            if (!follower_.startup)
            {
                if (dr_route_ != route_rest)
                    ac_arm_.cancelAllGoals();
                if (dr_route_ == route_raw_material_area && timer_.hasStarted())
                {
                    ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
                    ac_move_.cancelAllGoals();
                    boost::lock_guard<boost::mutex> lk(mtx_);
                    arm_initialized_ = arm_active_ = false;
                    move_initialized_ = move_active_ = false;
                }
                if (!timer_.hasStarted())
                    timer_.start();
            }
        }
        if (config.where != dr_route_)
        {
            if (follower_.startup)
            {
                ac_arm_.cancelAllGoals();
                if (dr_route_ == route_raw_material_area && timer_.hasStarted())
                {
                    ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
                    ac_move_.cancelAllGoals();
                    boost::lock_guard<boost::mutex> lk(mtx_);
                    arm_initialized_ = arm_active_ = false;
                    move_initialized_ = move_active_ = false;
                }
                ROS_WARN("Please shut down startup!");
            }
            boost::lock_guard<boost::mutex> lk(mtx_);
            dr_route_ = config.where;
        }
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
                boost::lock_guard<boost::mutex> lk(mtx_);
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
                boost::lock_guard<boost::mutex> lk(mtx_);
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
                boost::lock_guard<boost::mutex> lk(mtx_);
                move_active_ = false;
                move_initialized_ = false;
            }
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
            ac_move_.sendGoalAndWait(MoveGoal(), ros::Duration(5), ros::Duration(0.1));
            boost::lock_guard<boost::mutex> lk(mtx_);
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
            boost::lock_guard<boost::mutex> lk(mtx_);
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

    bool MotionController::go(Go::Request &req, Go::Response &res)
    {
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
