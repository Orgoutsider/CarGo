#include "motion_controller/line_follower.h"

namespace motion_controller
{
    LineFollower::LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : r_start_(131), r_end_(195),
          c_start_(16), c_end_(width_ - c_start_),
          mask_r_start_(35), mask_c_start_(100),
          Hough_threshold_(28),
          judge_line_(20), linear_velocity_(0.2),
          rho_thr_(5), theta_thr_(3),
          kp_(0.03), kd_(0.015),
          black_low_(0, 0, 0), black_up_(180, 255, 100),
          motor_status_(false), start_image_sub_(false)
    {
        it_ = std::shared_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(nh));
        pnh.param<std::string>("transport_hint", transport_hint_, "raw");
        cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_line", 1);
        pnh.param<bool>("param_modification", param_modification_, false);
        start_server_ = nh.advertiseService("Start", &LineFollower::_do_start_req, this);
        if (param_modification_)
        {
            image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1,
                                               &LineFollower::_image_callback, this,
                                               image_transport::TransportHints(transport_hint_));
            start_image_sub_ = true;
            dr_server_.setCallback(boost::bind(&LineFollower::_dr_callback, this, _1, _2));
        }
    }

    void LineFollower::_clean_lines(cv::Vec2f lines[], int num, double &rho_aver, double &theta_aver)
    {
        if (num == 0)
        {
            ROS_ERROR("num is zero!");
            return;
        }
        int num_ori = num;
        for (size_t i = 0; i < num_ori; i++)
        {
            double rho = lines[i][0];
            double theta = lines[i][1];
            if ((abs(rho_aver - rho) > rho_thr_ ||
                 abs(theta_aver - theta) > theta_thr_ / 180 * CV_PI) &&
                num > 1)
            {
                // 去除该直线，更新rho和theta
                rho_aver *= num;
                theta_aver *= num;
                rho_aver -= rho;
                theta_aver -= theta;
                num--;
                rho_aver /= num;
                theta_aver /= num;
            }
        }
    }

    bool LineFollower::_find_lines(cv_bridge::CvImagePtr &cv_image, geometry_msgs::Twist &twist)
    {
        using namespace cv;
        // 原先代码
        // 色彩分离查找车道线
        Mat srcF;
        cvtColor(cv_image->image, srcF, COLOR_BGR2HSV);
        inRange(srcF, black_low_, black_up_, srcF);
        if (param_modification_ && !motor_status_)
        {
            imshow("inRange", srcF);
            waitKey(1);
        }
        // 可能需要去除干扰轮廓,(5, 3)可能更好一点
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 1));
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
        Point p[6] = {Point(0, mask_r_start_), Point(mask_c_start_, 0),
                      Point(c_end_ - c_start_ - mask_c_start_, 0), Point(c_end_ - c_start_, mask_r_start_),
                      Point(c_end_ - c_start_, r_end_ - r_start_), Point(0, r_end_ - r_start_)};
        const Point *pp[] = {p};
        int n[] = {6};
        fillPoly(mask, pp, n, 1, Scalar(255));
        bitwise_and(srcF, mask, srcF);
        if (param_modification_ && !motor_status_)
        {
            imshow("mask", mask);
            imshow("line2", srcF);
            waitKey(1);
        }
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, Hough_threshold_, 0, 0); // 根据实际调整，夜晚40，下午50
        int minx, maxx, nowx;
        static double err_aver = 0, err_aver_last = 0;
        static bool flag = false;

        // 直线筛选参数
        int num[2] = {0}; // 筛选后的直线参数

        // 直线拟合参数 左0右1
        const int left = 0;
        const int right = 1;
        const int MAXN = lines.size();
        double rho_aver[2] = {0, 0};
        double theta_aver[2] = {0, 0};
        Vec2f lines_left_right[2][MAXN];
        double x[2] = {0};
        // ROS_INFO_STREAM(lines.size());
        Mat Hough(srcF.size(), CV_8UC3, Scalar::all(0));
        if (lines.size() > 1)
        {
            // 遍历直线求平均，分成左右两边
            for (Vec2f &line : lines)
            {
                double rho = line[0], theta = line[1];
                // if(abs(rho-rho_NUM)>rho_Derta && abs(theta-theta_NUM)>theta_Derta || abs(theta) < 3)//最后这个是滤去一些theta错误的直线，可以不加
                //     continue;
                if (theta < CV_PI / 2)
                {
                    rho_aver[left] += rho;
                    theta_aver[left] += theta;
                    lines_left_right[left][num[left]] = line;
                    num[left]++;
                    if (param_modification_ && !motor_status_)
                    {
                        // 左边绘制红色
                        double a = cos(theta), b = sin(theta);
                        double x0 = a * rho, y0 = b * rho;
                        Point pt1, pt2;
                        pt1.x = cvRound(x0 + 1000 * (-b));
                        pt1.y = cvRound(y0 + 1000 * (a));
                        pt2.x = cvRound(x0 - 1000 * (-b));
                        pt2.y = cvRound(y0 - 1000 * (a));
                        cv::line(Hough, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                    }
                }
                else
                {
                    rho_aver[right] += rho;
                    theta_aver[right] += theta;
                    lines_left_right[right][num[right]] = line;
                    num[right]++;
                    if (param_modification_ && !motor_status_)
                    {
                        // 右边绘制蓝色
                        double a = cos(theta), b = sin(theta);
                        double x0 = a * rho, y0 = b * rho;
                        Point pt1, pt2;
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
            if ((num[left] == 0) || (num[right] == 0))
            {
                // 即使没有车道线的参照，仍然前进
                twist = geometry_msgs::Twist();
                twist.linear.x = linear_velocity_;
                return false;
            }
            rho_aver[left] /= num[left];
            theta_aver[left] /= num[left];
            rho_aver[right] /= num[right];
            theta_aver[right] /= num[right];
            // 去除错误直线并拟合左右车道线
            _clean_lines(lines_left_right[left], num[left], rho_aver[left], theta_aver[left]);
            _clean_lines(lines_left_right[right], num[right], rho_aver[right], theta_aver[right]);

            Mat res = cv_image->image.clone();
            for (int i = left; i <= right; i++)
            {
                double a = cos(theta_aver[i]), b = sin(theta_aver[i]);
                x[i] = ((rho_aver[i] - b * judge_line_) / a);
                // 直线绘制
                if (param_modification_ && !motor_status_)
                {
                    double x0 = a * rho_aver[i], y0 = b * rho_aver[i];
                    Point pt1, pt2;
                    pt1.x = cvRound(x0 + 1000 * (-b));
                    pt1.y = cvRound(y0 + 1000 * (a));
                    pt2.x = cvRound(x0 - 1000 * (-b));
                    pt2.y = cvRound(y0 - 1000 * (a));
                    line(res, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                    line(res, Point(0, judge_line_), Point(c_end_ - c_start_, judge_line_), Scalar(255, 0, 0), 1, LINE_AA);
                    imshow("res", res);
                    waitKey(1);
                }
            }
            if (param_modification_ && !motor_status_)
            {
                imshow("res", res);
                waitKey(1);
            }
            double err = (x[left] + x[right]) / 2 - srcF.cols / 2;
            if (!flag)
            {
                flag = true;
                err_aver = err;
                err_aver_last = err;
            }
            // 滤除不正常的误差
            // 指数滑动平均法
            err_aver = (abs(err) > (c_end_ - c_start_) / 14.0) ? err_aver : 0.9 * err + 0.1 * err_aver;
            double d_err = err_aver_last - err_aver;
            twist = geometry_msgs::Twist();
            twist.linear.x = linear_velocity_;
            twist.angular.z = (err_aver != 0) ? -kp_ * err_aver - kd_ * d_err : 0;
            err_aver_last = err_aver;
            if (param_modification_ && motor_status_)
                ROS_INFO_STREAM("err:" << err_aver);
            // ROS_INFO_STREAM(minx << ' ' << maxx);
            return true;
        }
        else
        {
            // 即使没有车道线的参照，仍然前进
            twist = geometry_msgs::Twist();
            twist.linear.x = linear_velocity_;
            return false;
        }
    }

    // bool LineFollower::_find_road(cv_bridge::CvImagePtr &cv_image, geometry_msgs::Twist &twist)
    // {
    //     using namespace cv;
    //     Mat srcF, sure_bg, sure_fg, unknown, markers;
    //     cvtColor(cv_image->image, srcF, COLOR_BGR2GRAY);
    //     threshold(srcF, srcF, threshold_, 255, THRESH_BINARY_INV);
    //     if (param_modification_)
    //     {
    //         imshow("thresh", srcF);
    //         waitKey(1);
    //     }
    //     // noise removal
    //     Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    //     morphologyEx(srcF, srcF, MORPH_OPEN, element, Point(-1, -1), 2);
    //     // imshow("Foreground", srcF);
    //     // waitKey(1);
    //     // sure background area
    //     dilate(srcF, sure_bg, element, Point(-1, -1), 3);
    //     // imshow("Background", sure_bg);
    //     // waitKey(1);
    //     // 找到前景
    //     distanceTransform(srcF, srcF, DIST_L2, 5);
    //     float max_val = *std::max_element(srcF.begin<float>(), srcF.end<float>());
    //     threshold(srcF, sure_fg, 0.2 * max_val, 255, 0);
    //     sure_fg.convertTo(sure_fg, CV_8U);
    //     imshow("Threshold", sure_fg);
    //     waitKey(1);
    //     // 找到未知区域
    //     subtract(sure_bg, sure_fg, unknown);
    //     // Marker labelling
    //     connectedComponents(sure_fg, markers);
    //     // Add one to all labels so that sure background is not 0, but 1
    //     markers = markers + 1;
    //     // Now, mark the region of unknown with zero
    //     if (markers.size() != unknown.size())
    //     {
    //         ROS_ERROR("size error!");
    //         // 即使没有车道线的参照，仍然前进
    //         twist = geometry_msgs::Twist();
    //         twist.linear.x = linear_velocity_;
    //         return false;
    //     }
    //     for (int row = 0; row < markers.rows; row++)
    //     {
    //         int *markers_data = markers.ptr<int>(row);
    //         uchar *unknown_data = unknown.ptr<uchar>(row);
    //         for (int col = 0; col < markers.cols; col++)
    //         {
    //             if (*unknown_data == 255)
    //                 *markers_data = 0;
    //             unknown_data++;
    //             markers_data++;
    //         }
    //     }
    //     if (param_modification_)
    //     {
    //         // 中间图像，调试用
    //         Mat markers_copy = markers.clone();
    //         for (int row = 0; row < markers.rows; row++)
    //         {
    //             int *markers_data = markers.ptr<int>(row);
    //             int *copy_data = markers_copy.ptr<int>(row);
    //             for (int col = 0; col < markers.cols; col++)
    //             {
    //                 if (*markers_data == 0)
    //                     *copy_data = 150;
    //                 if (*markers_data == 1)
    //                     *copy_data = 0;
    //                 if (*markers_data > 1)
    //                     *copy_data = 255;
    //                 copy_data++;
    //                 markers_data++;
    //             }
    //         }
    //         markers_copy.convertTo(markers_copy, CV_8U);
    //         imshow("copy", markers_copy);
    //     }
    //     watershed(cv_image->image, markers);
    //     if (param_modification_)
    //     {
    //         for (int row = 0; row < markers.rows; row++)
    //         {
    //             int *markers_data = markers.ptr<int>(row);
    //             uchar *src_data = cv_image->image.ptr<uchar>(row);
    //             for (int col = 0; col < markers.cols; col++)
    //             {
    //                 // 原图标注红色
    //                 if (*markers_data == -1)
    //                 {
    //                     src_data[0] = 0;
    //                     src_data[1] = 0;
    //                     src_data[2] = 255;
    //                 }
    //                 markers_data++;
    //                 src_data += 3;
    //             }
    //         }
    //         imshow("res", cv_image->image);
    //         waitKey(1);
    //     }
    //     return true;
    // }

    void LineFollower::_image_callback(const sensor_msgs::ImageConstPtr &image_rect)
    {
        static int cnt = 0;
        if (cnt < 10) // 等待摄像头适应光线
        {
            cnt++;
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
        resize(cv_image->image, cv_image->image, Size(width_, height_));
        cv_image->image = cv_image->image(Range(r_start_, r_end_), Range(c_start_, c_end_));
        geometry_msgs::Twist twist;
        _find_lines(cv_image, twist);
        if (!param_modification_ || (param_modification_ && motor_status_))
            cmd_vel_publisher_.publish(twist);
        else
            cmd_vel_publisher_.publish(geometry_msgs::Twist());
    }

    void LineFollower::_dr_callback(lineConfig &config, uint32_t level)
    {
        if (!param_modification_)
            return;
        if (r_start_ != config.r_start)
        {
            if (config.r_start < r_end_ && judge_line_ < r_end_ - config.r_start && mask_r_start_ < r_end_ - config.r_start)
                r_start_ = config.r_start;
            else
                ROS_WARN("Assertion failed: r_start < r_end && judge_line < r_end - r_start && mask_r_start < r_end - r_start");
        }
        if (r_end_ != config.r_end)
        {
            if (config.r_end > r_start_ && judge_line_ < config.r_end - r_start_ && mask_r_start_ < config.r_end - r_start_)
                r_end_ = config.r_end;
            else
                ROS_WARN("Assertion failed: r_start < r_end && judge_line < r_end - r_start && mask_r_start < r_end - r_start");
        }
        if (c_start_ != config.c_start)
        {
            if (mask_c_start_ < ((width_ - config.c_start) - config.c_start) / 2.0)
            {
                c_start_ = config.c_start;
                c_end_ = width_ - c_start_;
            }
            else
                ROS_WARN("Assertion failed: mask_c_start < ((width - c_start) - c_start) / 2.0");
        }
        if (mask_r_start_ != config.mask_r_start)
        {
            if (config.mask_r_start < r_end_ - r_start_)
                mask_r_start_ = config.mask_r_start;
            else
                ROS_WARN("Assertion failed: mask_r_start < r_end - r_start");
        }
        if (mask_c_start_ != config.mask_c_start)
        {
            if (config.mask_c_start < (c_end_ - c_start_) / 2.0)
                mask_c_start_ = config.mask_c_start;
            else
                ROS_WARN("Assertion failed: mask_c_start < ((width - c_start) - c_start) / 2.0");
        }
        // if (threshold_ != config.threshold)
        //     threshold_ = config.threshold;
        if (Hough_threshold_ != config.Hough_threshold)
            Hough_threshold_ = config.Hough_threshold;
        if (judge_line_ != config.judge_line)
        {
            if (config.judge_line < r_end_ - r_start_)
                judge_line_ = config.judge_line;
            else
                ROS_WARN("Assertion failed: judge_line < r_end - r_start");
        }
        if (linear_velocity_ != config.linear_velocity)
        {
            linear_velocity_ = config.linear_velocity;
        }
        if (rho_thr_ != config.rho_thr)
            rho_thr_ = config.rho_thr;
        if (theta_thr_ != config.theta_thr)
            theta_thr_ = config.theta_thr;
        if (kp_ != config.kp)
            kp_ = config.kp;
        if (kd_ != config.kd)
            kd_ = config.kd;
        if (black_up_[2] != config.v_black_up)
            black_up_[2] = config.v_black_up;
        if (motor_status_ != config.motor_status)
        {
            motor_status_ = config.motor_status;
            if (motor_status_)
                cv::destroyAllWindows();
        }
    }

    bool LineFollower::_do_start_req(Start::Request &req, Start::Response &resp)
    {
        if (req.start)
        {
            if (!start_image_sub_)
            {
                start_image_sub_ = true;
                image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1,
                                                   &LineFollower::_image_callback, this,
                                                   image_transport::TransportHints(transport_hint_));
            }
            else
                return false;
        }
        else if (start_image_sub_)
        {
            start_image_sub_ = false;
            cmd_vel_publisher_.publish(geometry_msgs::Twist());
            image_subscriber_.shutdown();
        }
        else
            return false;
        return true;
    }
} // namespace motion_controller
