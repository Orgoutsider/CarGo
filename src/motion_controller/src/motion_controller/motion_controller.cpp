#include "motion_controller/motion_controller.h"

namespace motion_controller
{
    MotionController::MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : cnt_(0),
          r_start_(131), r_end_(195),
          c_start_(16), c_end_(width_ - c_start_),
          mask_c_start1_(100), mask_c_start2_(50), threshold_(40),
          black_low_(0, 0, 0), black_up_(180, 255, 94),
          yellow_low_(26, 43, 46), yellow_up_(50, 255, 255),
          grey_low_(0, 0, black_up_[2]), grey_up_(180, yellow_low_[1], 220),
          theta_thr_(10),
          y_goal_(48), y_ground_(3),
          cnt_tolerance_(2), y_thr_(3),
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
            start_image_subscriber(true);
        }
        vision_publisher = nh.advertise<Distance>("/vision_usb_cam", 5);
        timer_ = nh.createTimer(ros::Rate(3), &MotionController::_timer_callback, this, false, false);
        go_client_ = nh.advertiseService("Go", &MotionController::go, this);
        start_client_ = nh.serviceClient<Start>("Start");
    }

    bool MotionController::_turn()
    {
        if (!ac_move_.isServerConnected())
            ac_move_.waitForServer();
        cnt_ = 0;
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
            goal2.pose.theta = left_ ? CV_PI / 2 : -CV_PI / 2;
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
        goal.pose.theta = CV_PI;
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
            goal2.pose.theta = left_ ? CV_PI / 2 : -CV_PI / 2;
        // 转弯后前进一段距离
        goal2.pose.y = left_ ? 0.1 : -0.1;
        ac_move_.sendGoalAndWait(goal2, ros::Duration(15), ros::Duration(0.1));
        // 转弯后定时器将订阅关闭
        finish_turning_ = true;
        return true;
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

    bool MotionController::_color_judge(cv::Mat &img, bool note[], cv::Vec2f &line)
    {
        double rho = line[0];
        double theta = line[1];
        double a = cos(theta);
        double b = sin(theta);
        int y = cvRound((rho - a * img.cols / 2) / b) + r_start_ - height_ / 2;
        if (y >= 0 && y <= img.rows - 1)
        {
            if (note[y])
                return true;
        }
        else
            return false;
        int c_start = img.cols / 2 - 3;
        int c_end = img.cols / 2 + 3;
        int r_start1 = (y - 8 < 0) ? 0 : (y - 8 > img.rows ? img.rows : y - 8);
        int r_end1 = (y - 1 < 0) ? 0 : (y - 1 > img.rows ? img.rows : y - 1);
        double yellow = 0;
        // ROS_INFO_STREAM(r_start1 << " " << r_end1 << " " << img.rows);
        if (r_start1 < r_end1 && !(loop_ == 1 && where_is_car() == route_parking_area)) // 判黄色
        {
            cv::Mat roi = img(cv::Range(r_start1, r_end1), cv::Range(c_start, c_end)).clone();
            cv::inRange(roi, yellow_low_, yellow_up_, roi);
            cv::Scalar sum = cv::sum(roi);
            yellow = sum[0] / 255 / (roi.cols * roi.rows);
        }
        int r_start2 = (y + 1 < 0) ? 0 : (y + 1 > img.rows ? img.rows : y + 1);
        int r_end2 = (y + 8 < 0) ? 0 : (y + 8 > img.rows ? img.rows : y + 8);
        double grey = 0;
        // ROS_INFO_STREAM(r_start2 << " " << r_end2 << " " << img.rows);
        if (r_start2 < r_end2) // 判灰色
        {
            cv::Mat roi = img(cv::Range(r_start2, r_end2), cv::Range(c_start, c_end)).clone();
            cv::inRange(roi, grey_low_, grey_up_, roi);
            cv::Scalar sum = cv::sum(roi);
            grey = sum[0] / 255 / (roi.cols * roi.rows);
        }
        if (param_modification_ && !motor_status_)
            ROS_INFO_STREAM("y:" << y << "grey:" << grey << "yellow:" << yellow);
        if (loop_ == 1 && where_is_car() == route_parking_area)
            return grey > 0.5;
        note[y] = (grey > 0.5 && yellow > 0.5);
        return note[y];
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
        Mat res = cv_image->image(Range(height_ / 2, height_), Range(0, width_)).clone();
        Mat hsv;
        cvtColor(res, hsv, COLOR_BGR2HSV);
        Mat srcF = hsv(Range(r_start_ - height_ / 2, r_end_ - height_ / 2), Range(c_start_, c_end_)).clone();
        // 原先代码
        // 色彩分离查找车道线
        inRange(srcF, black_low_, black_up_, srcF);
        if (param_modification_ && !motor_status_)
        // if (where_is_car() == route_semi_finishing_area)
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
        // if (where_is_car() == route_semi_finishing_area)
        {
            imshow("mask", mask);
            imshow("line2", srcF);
            waitKey(1);
            Mat yellow, grey;
            inRange(hsv, yellow_low_, yellow_up_, yellow);
            inRange(hsv, grey_low_, grey_up_, grey);
            imshow("yellow", yellow);
            imshow("grey", grey);
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
            bool note[width_ / 2] = {0};
            double y[lines.size()] = {0};
            if (param_modification_)
            {
                line(res, Point(0, y_ground_), Point(width_, y_ground_), Scalar(0, 255, 0), 1, LINE_AA);
                line(res, Point(0, y_goal_), Point(width_, y_goal_), Scalar(255, 0, 0), 1, LINE_AA);
            }
            bool flag_cnt = false; // 是否识别到正确线
            for (Vec2f &line : lines)
            {
                double rho = line[0], theta = line[1];
                double a = cos(theta), b = sin(theta);

                // 计算斜率
                // ROS_INFO_STREAM("theta_d:" << theta_d << " cnt_:" << cnt_);
                double theta_d = theta * 180 / CV_PI;
                if (90 - theta_thr_ < theta_d && theta_d < 90 + theta_thr_ && _color_judge(hsv, note, line))
                {
                    if (param_modification_ && !motor_status_)
                    // if (where_is_car() == route_semi_finishing_area)
                    {
                        plot_line(Hough, rho, theta, Scalar(0, 0, 255)); // 红色表示正确识别
                    }
                    if (cnt_ < cnt_tolerance_)
                    {
                        if (!flag_cnt)
                            flag_cnt = true;
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
                    if (param_modification_ && !motor_status_)
                    // if (where_is_car() == route_semi_finishing_area)
                    {
                        plot_line(Hough, rho, theta, Scalar(255, 0, 0)); // 蓝色表示错误识别
                    }
                }
            }
            if (cnt_ < cnt_tolerance_)
            {
                if (flag_cnt)
                {
                    if (cnt_ == 0 && !param_modification_)
                        start_line_follower(false);
                    cnt_++;
                    // if (where_is_car() == route_semi_finishing_area)
                    // {
                    //     imshow("raw", cv_image->image);
                    //     imshow("line", srcF);
                    //     imshow("Hough", Hough);
                    //     start_line_follower(false);
                    //     waitKey();
                    // }
                    ROS_INFO_STREAM("cnt: " << cnt_);
                    if (cnt_ >= cnt_tolerance_)
                    {
                        if (timer_.hasStarted())
                            timer_.stop();
                        if (!param_modification_)
                        {
                            ROS_INFO("Turning corners ...");
                            err_cnt = 0;
                            return;
                        }
                    }
                }
                else if (cnt_ != 0) // 必须连续检测到
                {
                    cnt_ = 0;
                    if (!param_modification_)
                        start_line_follower(true);
                }
            }
            if (param_modification_ && !motor_status_)
            // if (where_is_car() == route_semi_finishing_area)
            {
                imshow("Hough", Hough);
                waitKey(1);
            }
            if (cnt_ >= cnt_tolerance_ && tot != 0) // 弯道pid
            {
                _clean_lines(y, y_sum, tot);
                double y_now = y_sum / tot + r_start_ - height_ / 2.0;
                if (!flag)
                {
                    y_aver = y_now;
                    flag = true;
                }
                // ROS_INFO_STREAM(y_now);
                y_aver = 0.9 * y_now + 0.1 * y_aver;
                double distance;
                if (y_ground_ < y_aver - 0.01 || y_ground_ < y_goal_)
                    distance = 1 / (1.0 / y_ground_ - 1.0 / y_aver) - 1 / (1.0 / y_ground_ - 1.0 / y_goal_);
                else
                {
                    ROS_WARN("y_ground or y_goal is invalid!");
                    return;
                }
                if (abs(distance) < distance_thr_)
                {
                    if (!param_modification_)
                    {
                        err_cnt++;
                        ROS_INFO_STREAM("err_cnt: " << err_cnt);
                    }
                    else if (!motor_status_)
                    {
                        // ROS_INFO("Corner is in front of my car.");
                        cnt_ = 0;
                    }
                    if (err_cnt > 2)
                    {
                        vision_publisher.publish(Distance());
                        // 客户端转弯，顺时针对应右转
                        if (!_turn()) // 出现了错误识别，重新开启弯道节点
                        {
                            ROS_WARN("Corner detection error, restart ...");
                            return;
                        }
                        if (!timer_.hasStarted())
                            timer_.start();
                        if (!param_modification_)
                            start_line_follower(true);
                    }
                    else if (param_modification_ && !motor_status_)
                    {
                        cnt_ = 0;
                        vision_publisher.publish(Distance());
                    }
                    else
                    {
                        Distance msg;
                        msg.distance = distance;
                        msg.header = image_rect->header;
                        vision_publisher.publish(msg);
                    }
                }
                else
                {
                    // 必须连续达到要求
                    if (err_cnt != 0 || param_modification_)
                        err_cnt = 0;
                    if (param_modification_ && !motor_status_)
                    {
                        // ROS_INFO("Corner is in front of my car.");
                        cnt_ = 0;
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
                    }
                }
            }
            if (param_modification_)
            {
                imshow("res", res);
                waitKey(1);
            }
        }
        // 应急方案，使用全局定位转弯，需要调试
        // else if (!param_modification_)
        // {
        //     if (_turn_by_position())
        //     {
        //         if (!timer_.hasStarted())
        //             timer_.start();
        //         if (cnt_ > cnt_tolerance_)
        //         {
        //             start_line_follower(true);
        //         }
        //         cnt_ = 0;
        //     }
        // }
    }

    void MotionController::_timer_callback(const ros::TimerEvent &event)
    {
        if (get_position())
        {
            // 转弯结束关闭转弯订阅
            // ROS_INFO_STREAM("x: " << x_ << " y: " << y_);
            if (finish_turning_)
            {
                if (!can_turn())
                    start_image_subscriber(false);
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
        // return;
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
        if (yellow_low_[0] != config.h_yellow_low)
            yellow_low_[0] = config.h_yellow_low;
        if (yellow_up_[0] != config.h_yellow_up)
            yellow_up_[0] = config.h_yellow_up;
        if (yellow_low_[1] != config.s_yellow_low)
            yellow_low_[1] = config.s_yellow_low;
        if (grey_up_[1] != config.s_yellow_low)
            grey_up_[1] = config.s_yellow_low;
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
        if (where_is_car() == route_raw_material_area && loop_ == 1)
            _U_turn();
    }

    void MotionController::_feedback_callback() {}

    void MotionController::_done_callback(const actionlib::SimpleClientGoalState &state, const my_hand_eye::ArmResultConstPtr &result)
    {
        if (where_is_car() == route_raw_material_area || where_is_car() == route_roughing_area)
            start_image_subscriber(true);
        else if (where_is_car() == route_semi_finishing_area)
        {
            if (loop_ == 0)
                _U_turn();
            start_image_subscriber(true);
        }
        else if (where_is_car() == route_parking_area)
        {
            if (!ac_move_.isServerConnected())
                ac_move_.waitForServer();
            // 先移动到停车区的下侧
            get_position();
            motion_controller::MoveGoal goal1;
            goal1.pose.x = y_ - (y_road_up_up_ + length_car_ / 2);
            goal1.pose.theta = CV_PI / 2;
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
        start_line_follower(true);
    }

    bool MotionController::set_position(double x, double y, double theta)
    {
        x_ = x;
        y_ = y;
        theta_ = (theta > -CV_PI) ? theta + CV_PI : (theta <= -CV_PI ? theta - CV_PI : theta);
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
        geometry_msgs::PointStamped point_odom;
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            geometry_msgs::TransformStamped tfs = buffer_.lookupTransform("odom_combined",
                                                                          "base_footprint", ros::Time(0));
            x_ = delta_x_ + tfs.transform.translation.x;
            y_ = delta_y_ + tfs.transform.translation.y;
            theta_ = delta_theta_ + atan2(tfs.transform.rotation.z, tfs.transform.rotation.w) * 2;
            theta_ = (theta_ > -CV_PI) ? theta_ + CV_PI : (theta_ <= -CV_PI ? theta_ - CV_PI : theta_);
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
        else
            ROS_WARN("Invalid request! LineFollower cannot start/shutdown.");
        return flag;
    }

    void MotionController::start_image_subscriber(bool start)
    {
        if (start)
        {
            cnt_ = 0;
            image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1,
                                               &MotionController::_image_callback, this,
                                               image_transport::TransportHints(transport_hint_));
            ROS_INFO("start image subscriber.");
        }
        else
        {
            image_subscriber_.shutdown();
            ROS_INFO("shutdown image subscriber.");
        }
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
        start_line_follower(true);
        timer_.start();
        return true;
    }

    void MotionController::plot_line(cv::Mat &mat, double rho, double theta, cv::Scalar color)
    {
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        cv::line(mat, pt1, pt2, color, 1, cv::LINE_AA);
    }
} // namespace motion_controller
