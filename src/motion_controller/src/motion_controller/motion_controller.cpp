#include "motion_controller/motion_controller.h"

namespace motion_controller
{
    MotionController::MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : r_start_(131), r_end_(195),
          c_start_(16), c_end_(width_ - c_start_),
          mask_c_start1_(100), mask_c_start2_(50), threshold_(40),
          black_low_(0, 0, 0), black_up_(180, 255, 105), theta_thr_(8),
          y_goal_(48), y_ground_(3),
          cnt_tolerance_(15), y_thr_(3),
          distance_thr_(0.05), motor_status_(false),
          delta_x_(0), delta_y_(0), finish_turning_(false),
          listener_(buffer_),
          ac_move_(nh, "Move", true), ac_arm_(nh, "Arm", true)
    {
        nh.setParam("/width_road", width_road_);
        pnh.param<bool>("param_modification", param_modification_, false);
        pnh.param<std::string>("transport_hint", transport_hint_, "raw");
        it_ = std::shared_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(nh));
        if (param_modification_)
        {
            dr_server_.setCallback(boost::bind(&MotionController::_dr_callback, this, _1, _2));
            image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1,
                                               &MotionController::_image_callback, this,
                                               image_transport::TransportHints(transport_hint_));
        }
        vision_publisher = nh.advertise<Distance>("/vision_usb_cam", 5);
        timer_ = nh.createTimer(ros::Rate(3), &MotionController::_timer_callback, this, false, false);
        go_client_ = nh.advertiseService("Go", &MotionController::go, this);
        start_client_ = nh.serviceClient<Start>("Start");
    }

    void MotionController::_turn()
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        // 矫正pid结束位置与转弯位置的8cm误差
        motion_controller::MoveGoal goal1;
        goal1.pose.x = 0.08;
        ac_move_.sendGoalAndWait(goal1, ros::Duration(5), ros::Duration(0.1));
        motion_controller::MoveGoal goal2;
        goal2.pose.theta = left_ ? CV_PI / 2 : -CV_PI / 2;
        // 转弯后前进一段距离
        if (get_position())
        {
            double len = length_route();
            if (!can_turn() && len <= 0.8) // 下一路段有任务
            {
                goal2.pose.y = left_ ? len : -len;
            }
            else
                goal2.pose.y = left_ ? 0.5 : -0.5;
        }
        else
            goal2.pose.y = left_ ? 0.5 : -0.5;
        ac_move_.sendGoalAndWait(goal2, ros::Duration(15), ros::Duration(0.1));
        // 转弯后定时器将订阅关闭
        finish_turning_ = true;
    }

    void MotionController::_U_turn()
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        motion_controller::MoveGoal goal;
        goal.pose.theta = CV_PI;
        ac_move_.sendGoalAndWait(goal, ros::Duration(15), ros::Duration(0.1));
        // 需要改变之后的转弯方向
        left_ = !left_;
    }

    void MotionController::_turn_by_position()
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        if (get_position())
        {
            // 矫正误差
            motion_controller::MoveGoal goal1;
            goal1.pose.x = length_corner();
            if (goal1.pose.x)
                ac_move_.sendGoalAndWait(goal1, ros::Duration(15), ros::Duration(0.1));
        }
        motion_controller::MoveGoal goal2;
        goal2.pose.theta = left_ ? CV_PI / 2 : -CV_PI / 2;
        // 转弯后前进一段距离
        if (get_position())
        {
            double len = length_route();
            if (!can_turn() && len <= 0.8) // 下一路段有任务
            {
                goal2.pose.y = left_ ? len : -len;
            }
            else
                goal2.pose.y = left_ ? 0.5 : -0.5;
        }
        else
            goal2.pose.y = left_ ? 0.5 : -0.5;
        ac_move_.sendGoalAndWait(goal2, ros::Duration(15), ros::Duration(0.1));
        // 转弯后定时器将订阅关闭
        finish_turning_ = true;
    }

    void MotionController::_clean_lines(double y[], double &y_sum, int &tot)
    {
        if (tot <= 0)
        {
            ROS_ERROR("tot must be greater than zero");
            return;
        }
        int tot_ori = tot;
        for (int i = 0; i < tot; i++)
        {
            if (abs(y_sum / tot - y[i]) > y_thr_)
            {
                y_sum -= y[i];
                tot--;
            }
        }
        return;
    }

    void MotionController::_image_callback(const sensor_msgs::ImageConstPtr &image_rect)
    {
        static int count = 0;
        if (count < 15) // 等待摄像头适应光线
        {
            count++;
            return;
        }
        using namespace cv;
        static int cnt = 0;
        cv_bridge::CvImagePtr cv_image;
        try
        {
            cv_image = cv_bridge::toCvCopy(image_rect, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if (!(cv_image->image.data))
        {
            ROS_ERROR("No data!");
            return;
        }
        resize(cv_image->image, cv_image->image, Size(width_, height_));
        Mat srcF = cv_image->image(Range(r_start_, r_end_), Range(c_start_, c_end_)).clone();
        // 原先代码
        // 色彩分离查找车道线
        cvtColor(srcF, srcF, COLOR_BGR2HSV);
        inRange(srcF, black_low_, black_up_, srcF);
        if (param_modification_ && !motor_status_)
        {
            imshow("inRange", srcF);
            waitKey(1);
        }
        // 可能需要去除干扰轮廓,(5, 3)可能更好一点
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 1));
        // morphologyEx(srcF, srcF, MORPH_CLOSE, element);
        dilate(srcF, srcF, element);
        if (param_modification_ && !motor_status_)
        {
            imshow("line1", srcF);
            waitKey(1);
        }
        Canny(srcF, srcF, 50, 100, 3);

        // 然后在这里接入轮廓的一些筛选和排除，然后进行直线识别
        // mask掩膜
        Mat mask = Mat::zeros(srcF.size(), CV_8UC1);
        Point p[4] = {Point(mask_c_start1_, 0), Point(c_end_ - c_start_ - mask_c_start1_, 0),
                      Point(c_end_ - c_start_ - mask_c_start2_, r_end_ - r_start_),
                      Point(mask_c_start2_, r_end_ - r_start_)};
        const Point *pp[] = {p};
        int n[] = {4};
        fillPoly(mask, pp, n, 1, Scalar(255));
        bitwise_and(srcF, mask, srcF);
        if (param_modification_ && !motor_status_)
        {
            imshow("mask", mask);
            imshow("line2", srcF);
            waitKey(1);
        }
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, threshold_, 0, 0); // 根据实际调整
        static int err_cnt = 0;                                    // 防错误识别计数器
        if (lines.size())
        {
            int tot = 0;
            double y_sum = 0;
            static double y_aver = 0;
            static bool flag = false;
            Mat Hough = Mat::zeros(srcF.size(), CV_8UC3);
            Mat res = cv_image->image(Range(height_ / 2, height_), Range(0, width_)).clone();
            double y[lines.size()] = {0};
            if (param_modification_)
            {
                line(res, Point(0, y_ground_), Point(width_, y_ground_), Scalar(0, 255, 0), 1, LINE_AA);
                line(res, Point(0, y_goal_), Point(width_, y_goal_), Scalar(255, 0, 0), 1, LINE_AA);
            }
            for (Vec2f &line : lines)
            {
                double rho = line[0], theta = line[1];
                double a = cos(theta), b = sin(theta);

                // 计算斜率
                // ROS_INFO_STREAM("theta_d:" << theta_d << " cnt:" << cnt);
                double theta_d = theta * 180 / CV_PI;
                if (90 - theta_thr_ < theta_d && theta_d < 90 + theta_thr_)
                {
                    if (param_modification_ && !motor_status_)
                    {
                        Point pt1, pt2;
                        double x0 = a * rho, y0 = b * rho;
                        pt1.x = cvRound(x0 + 1000 * (-b));
                        pt1.y = cvRound(y0 + 1000 * (a));
                        pt2.x = cvRound(x0 - 1000 * (-b));
                        pt2.y = cvRound(y0 - 1000 * (a));
                        cv::line(Hough, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                    }
                    if (cnt < cnt_tolerance_)
                    {
                        cnt++;
                        if (cnt >= cnt_tolerance_)
                        {
                            if (timer_.hasStarted())
                                timer_.stop();
                            if (!param_modification_)
                            {
                                ROS_INFO("Turning corners ...");
                                start_line_follower(false);
                                return;
                            }
                            break;
                        }
                        continue;
                    }
                    else
                    {
                        y[tot] = (rho - a * srcF.cols / 2) / b;
                        y_sum += y[tot];
                        tot++;
                    }
                }
                else
                {
                    cnt = 0;
                    if (param_modification_ && !motor_status_)
                    {
                        Point pt1, pt2;
                        double x0 = a * rho, y0 = b * rho;
                        pt1.x = cvRound(x0 + 1000 * (-b));
                        pt1.y = cvRound(y0 + 1000 * (a));
                        pt2.x = cvRound(x0 - 1000 * (-b));
                        pt2.y = cvRound(y0 - 1000 * (a));
                        cv::line(Hough, pt1, pt2, Scalar(255, 0, 0), 1, LINE_AA);
                    }
                }
            }
            if (param_modification_ && !motor_status_)
            {
                imshow("Hough", Hough);
                waitKey(1);
            }
            if (cnt >= cnt_tolerance_ && tot != 0) // 弯道pid
            {
                _clean_lines(y, y_sum, tot);
                double y_now = y_sum / tot + r_start_ - height_ / 2.0;
                if (!flag)
                {
                    y_aver = y_now;
                    flag = true;
                }
                // ROS_INFO_STREAM(y_now);
                if (abs(y_now - y_aver) > (r_end_ - r_start_) / 6)
                {
                    err_cnt++;
                    ROS_WARN("Line might be out of limit!");
                }
                else
                    y_aver = 0.9 * y_now + 0.1 * y_aver;
                double distance;
                if (y_ground_ < y_aver - 0.01 || y_ground_ < y_goal_)
                    distance = 1 / (1.0 / y_ground_ - 1.0 / y_aver) - 1 / (1.0 / y_ground_ - 1.0 / y_goal_);
                else
                {
                    ROS_WARN("y_ground or y_goal is invalid!");
                    return;
                }
                if (abs(distance) < distance_thr_ && !param_modification_)
                {
                    cnt = 0;
                    err_cnt = 0;
                    vision_publisher.publish(Distance());
                    // 客户端转弯，顺时针对应右转
                    _turn();
                    if (!timer_.hasStarted())
                        timer_.start();
                    if (!param_modification_)
                        start_line_follower(true);
                }
                else if (param_modification_ && !motor_status_)
                {
                    // ROS_INFO("Corner is in front of my car.");
                    cnt = 0;
                    // 即停
                    vision_publisher.publish(Distance());
                    // ROS_INFO_STREAM("distance: " << distance);
                }
                else // (param_modification_ && motor_status_) || !param_modification_
                {
                    if (param_modification_)
                    {
                        line(res, Point(0, y_now), Point(width_, y_now), Scalar(0, 0, 255), 1, LINE_AA);
                        ROS_INFO_STREAM("distance: " << distance);
                    }
                    Distance msg;
                    msg.distance = distance;
                    msg.header = image_rect->header;
                    vision_publisher.publish(msg);
                    err_cnt++;
                }
            }
            else if (cnt >= cnt_tolerance_ && !param_modification_)
            {
                // 可能是超出了能够检测线的范围，使用全局定位信息转弯
                ROS_WARN("Line might be out of limit!");
                vision_publisher.publish(Distance());
                err_cnt++;
            }
            if (param_modification_)
            {
                imshow("res", res);
                waitKey(1);
            }
        }
        else if (cnt >= cnt_tolerance_ && !param_modification_)
        {
            // 可能是超出了能够检测线的范围，使用全局定位信息转弯
            ROS_WARN("Line might be out of limit!");
            vision_publisher.publish(Distance());
        }
        else if (cnt >= cnt_tolerance_ && param_modification_ && motor_status_)
        {
            ROS_WARN("Line might be out of limit!");
            vision_publisher.publish(Distance());
        }
        if (err_cnt > 10)
        {
            // 使用全局定位信息转弯
            cnt = 0;
            err_cnt = 0;
            vision_publisher.publish(Distance());
            // 客户端转弯，顺时针对应右转
            _turn_by_position();
            if (!timer_.hasStarted())
                timer_.start();
            if (!param_modification_)
                start_line_follower(true);
        }
    }

    void MotionController::_timer_callback(const ros::TimerEvent &event)
    {
        if (get_position())
        {
            // 转弯结束关闭转弯订阅
            ROS_INFO_STREAM("x: " << x_ << " y: " << y_);
            if (finish_turning_ && !can_turn())
                image_subscriber_.shutdown();
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
                    ROS_ERROR("where_is_car returns invalid value!");
                    return;
                }
                ac_arm_.sendGoal(goal, boost::bind(&MotionController::_done_callback, this, _1, _2),
                                 boost::bind(&MotionController::_active_callback, this),
                                 boost::bind(&MotionController::_feedback_callback, this));
                doing();
                start_line_follower(false);
            }
        }
    }

    void MotionController::_dr_callback(motion_controller::cornersConfig &config, uint32_t level)
    {
        if (!param_modification_)
            return;
        if (r_start_ != config.r_start)
        {
            if (config.r_start < r_end_)
                r_start_ = config.r_start;
            else
                ROS_WARN("Assertion failed: r_start < r_end");
        }
        if (r_end_ != config.r_end)
        {
            if (config.r_end > r_start_)
                r_end_ = config.r_end;
            else
                ROS_WARN("Assertion failed: r_start < r_end");
        }
        if (c_start_ != config.c_start)
        {
            if (mask_c_start1_ < ((width_ - config.c_start) - config.c_start) / 2.0 &&
                mask_c_start2_ < ((width_ - config.c_start) - config.c_start) / 2.0)
            {
                c_start_ = config.c_start;
                c_end_ = width_ - c_start_;
            }
            else
                ROS_WARN("Assertion failed: mask_c_start < (c_end - c_start) / 2.0");
        }
        if (mask_c_start1_ != config.mask_c_start1)
        {
            if (config.mask_c_start1 < (c_end_ - c_start_) / 2.0)
                mask_c_start1_ = config.mask_c_start1;
            else
                ROS_WARN("Assertion failed: mask_c_start < (c_end - c_start) / 2.0");
        }
        if (mask_c_start2_ != config.mask_c_start2)
        {
            if (config.mask_c_start2 < (c_end_ - c_start_) / 2.0)
                mask_c_start2_ = config.mask_c_start2;
            else
                ROS_WARN("Assertion failed: mask_c_start < (c_end - c_start) / 2.0");
        }
        if (threshold_ != config.threshold)
            threshold_ = config.threshold;
        if (black_up_[2] != config.v_black_up)
            black_up_[2] = config.v_black_up;
        if (theta_thr_ != config.theta_thr)
            theta_thr_ = config.theta_thr;
        if (y_thr_ != config.y_thr)
            y_thr_ = config.y_thr;
        if (y_goal_ != config.y_goal)
        {
            if (config.y_goal > y_ground_)
                y_goal_ = config.y_goal;
            else
                ROS_WARN("Assertion failed: y_goal > y_ground");
        }
        if (y_ground_ != config.y_ground)
        {
            if (config.y_ground < y_goal_)
                y_ground_ = config.y_ground;
            else
                ROS_WARN("Assertion failed: y_goal > y_ground");
        }
        if (cnt_tolerance_ != config.cnt_tolerance)
            cnt_tolerance_ = config.cnt_tolerance;
        if (distance_thr_ != config.distance_thr)
            distance_thr_ = config.distance_thr;
        if (motor_status_ != config.motor_status)
        {
            motor_status_ = config.motor_status;
            if (motor_status_)
                cv::destroyAllWindows();
        }
    }

    void MotionController::_active_callback()
    {
        if (timer_.hasStarted())
            timer_.stop();
    }

    void MotionController::_feedback_callback() {}

    void MotionController::_done_callback(const actionlib::SimpleClientGoalState &state, const my_hand_eye::ArmResultConstPtr &result)
    {
        if (where_is_car() == route_raw_material_area)
        {
            if (loop_ == 1)
                _U_turn();
            image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1,
                                               &MotionController::_image_callback, this,
                                               image_transport::TransportHints(transport_hint_));
        }
        else if (where_is_car() == route_roughing_area)
            image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1,
                                               &MotionController::_image_callback, this,
                                               image_transport::TransportHints(transport_hint_));
        else if (where_is_car() == route_semi_finishing_area)
        {
            if (loop_ == 0)
                _U_turn();
            image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1,
                                               &MotionController::_image_callback, this,
                                               image_transport::TransportHints(transport_hint_));
        }
        else if (where_is_car() == route_parking_area)
        {
            if (!ac_move_.isServerConnected())
                ac_move_.waitForServer();
            // 先移动到停车区的下侧
            motion_controller::MoveGoal goal1;
            goal1.pose.x = y_parking_area_ - (y_road_up_up_ + length_car_ / 2);
            goal1.pose.theta = CV_PI / 2;
            goal1.pose.y = -(width_road_ / 2 - length_car_ / 2);
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
        start_line_follower(true);
    }

    bool MotionController::set_position(double x, double y)
    {
        x_ = x;
        y_ = y;
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            delta_x_ = x - tfs.transform.translation.x;
            delta_y_ = y - tfs.transform.translation.y;
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
        geometry_msgs::PointStamped point_odom;
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            x_ = delta_x_ + tfs.transform.translation.x;
            y_ = delta_y_ + tfs.transform.translation.y;
        }
        catch (const std::exception &e)
        {
            ROS_WARN("get_position exception:%s", e.what());
            return false;
        }
        return true;
    }

    bool MotionController::start_line_follower(bool start)
    {
        if (!start_client_.exists())
            start_client_.waitForExistence();
        Start srv;
        srv.request.start = start;
        bool flag = start_client_.call(srv);
        if (flag)
        {
            if (start)
                ROS_INFO("start LineFollower.");
            else
                ROS_INFO("shut down LineFollower.");
        }
        return flag;
    }

    bool MotionController::go(Go::Request &req, Go::Response &resp)
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        if (!set_position(length_car_ / 2, length_car_ / 2))
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
        ac_move_.sendGoalAndWait(goal2, ros::Duration(15), ros::Duration(0.1));
        start_line_follower(true);
        timer_.start();
        return true;
    }
} // namespace motion_controller
