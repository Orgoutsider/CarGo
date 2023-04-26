#include "motion_controller/line_follower.h"

namespace motion_controller
{
    LineFollower::LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : r_start_(100), r_end_(160),
          c_start_(20), c_end_(width_ - c_start_), threshold_(50),
          judge_line_(10), linear_velocity_(0.2),
          kp_(0.0015), kd_(0.001),
          black_low_(0, 0, 0), black_up_(180, 255, 100), motor_status_(false)
    {
        it_ = std::shared_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(nh));
        std::string transport_hint;
        pnh.param<std::string>("transport_hint", transport_hint, "raw");
        image_subscriber_ = it_->subscribe("/usb_cam/image_raw", 1, &LineFollower::_image_callback, this, image_transport::TransportHints(transport_hint));
        cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_line", 1);
        pnh.param<bool>("param_modification", param_modification_, false);
        if (param_modification_)
            server_.setCallback(boost::bind(&LineFollower::_dr_callback, this, _1, _2));
    }

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
            cv_image = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Mat srcF;
        resize(cv_image->image, srcF, Size(width_, height_));
        srcF = srcF(Range(r_start_, r_end_), Range(c_start_, c_end_));
        // srcF.convertTo(srcF, CV_8UC3);
        cvtColor(srcF, srcF, COLOR_BGR2HSV);
        inRange(srcF, black_low_, black_up_, srcF);
        // threshold(srcF, srcF, 100, 255, THRESH_BINARY);
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 1));
        // morphologyEx(srcF, srcF, MORPH_CLOSE, element);
        dilate(srcF, srcF, element);
        Canny(srcF, srcF, 50, 100, 3);
        if (param_modification_)
        {
            imshow("line", srcF);
            waitKey(1);
        }
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, threshold_, 0, 0); // 根据实际调整，夜晚40，下午50
        Mat res(srcF.size(), CV_8UC3, Scalar::all(0));             // int flag1 = 0;
        int minx, maxx, nowx;
        static int last_err = 0, tmp = 0;
        // int bar = 400, last_maxx, last_minx;
        // int err_t = 0;
        geometry_msgs::Twist twist;

        //直线筛选参数
        //double rho_Derta = ;//差值阈值
        //double theta_Derta = ;//差值阈值
        double rho_NUM = 0;//平均值
        double theta_NUM = 0;//平均值
        int NUM_of_RightLINE = 0;//筛选后的直线参数

        //直线拟合参数 左0右1
        double rho[2] = {0, 0};
        double theta[2] = {0, 0};

        if (lines.size() > 1)
        {
            //求rho和theta平均
            for (int i = 0; i < lines.size(); i++)
            {

                rho_NUM += lines[i][0];
                theta_NUM += lines[i][1];
            }
            rho_NUM /= lines.size();
            theta_NUM /= lines.size();

            //去除错误直线并拟合左右车道线
            for(int i = 0; i < lines.size(); i++)
            {
                double rho = lines[i][0], theta = lines[i][1];
                //if(abs(rho-rho_NUM)>rho_Derta && abs(theta-theta_NUM)>theta_Derta || abs(theta) < 3)//最后这个是滤去一些theta错误的直线，可以不加
                //    continue;
                if(theta > 0)
                {
                    rho[0] += rho;
                    theta[0] += theta;
                }
                else if(theta < 0)
                {
                    rho[1] += rho;
                    theta[1] += theta;
                }
                NUM_of_RightLINE++;
            }
            rho[0] /= NUM_of_RightLINE;
            theta[0] /= NUM_of_RightLINE;
            rho[1] /= NUM_of_RightLINE;
            theta[1] /= NUM_of_RightLINE;
            //直线绘制
            for(i = 0; i < 2; i++)
            {
                double a = cos(theta[i]), b = sin(theta[i]);
                double x0 = a * rho[i], y0 = b * rho[i];
                if (param_modification_)
                {
                    Point pt1, pt2;
                    pt1.x = cvRound(x0 + 1000 * (-b));
                    pt1.y = cvRound(y0 + 1000 * (a));
                    pt2.x = cvRound(x0 - 1000 * (-b));
                    pt2.y = cvRound(y0 - 1000 * (a));
                    line(res, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                    line(res, Point(0, judge_line_), Point(c_end_ - c_start_, judge_line_), Scalar(255, 0, 0), 1, LINE_AA);
                }
            }
            imshow("res", res);
            waitKey(1);

            nowx = cvRound((rho - b * judge_line_) / a);
            if (i = 0)
            {
                minx = maxx = nowx;
                flag_for = false;
            }

            minx = (nowx < minx) ? nowx : minx;
            maxx = (nowx > maxx) ? nowx : maxx;
            int err = (minx + maxx) / 2 - srcF.cols / 2;
            err = (abs(err) > 40) ? last_err : err;
            if (tmp = 0)
            {
                tmp = 1;
                last_err = err;
            }
            int d_err = err - last_err;
            twist.linear.x = linear_velocity_;
            twist.angular.z = (err != 0) ? -kp_ * err - kd_ * d_err : 0;
            last_err = err;
            // ROS_INFO_STREAM("err" << err << " size:" << lines.size());
            // ROS_INFO_STREAM(minx << ' ' << maxx);
        }
        else
        {
            twist.linear.x = linear_velocity_;
        }
        // ROS_INFO_STREAM(lines.size());
        if (!param_modification_ || (param_modification_ && motor_status_))
            cmd_vel_publisher_.publish(twist);
    }

    void LineFollower::_dr_callback(lineConfig &config, uint32_t level)
    {
        if (!param_modification_)
            return;
        if (r_start_ != config.r_start)
            r_start_ = config.r_start;
        if (r_end_ != config.r_end)
        {
            if (config.r_end > r_start_)
                r_end_ = config.r_end;
            else    
                ROS_WARN("r_end must be larger than r_start!");
        }
        if (c_start_ != config.c_start)
        {
            c_start_ = config.c_start;
            c_end_ = width_ - c_start_;
        }
        if (threshold_ != config.threshold)
            threshold_ = config.threshold;
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
        if (kp_ != config.kp)
            kp_ = config.kp;
        if (kd_ != config.kd)
            kd_ = config.kd;
        if (black_up_[0] != config.h_black_up)
            black_up_[0] = config.h_black_up;
        if (black_up_[2] != config.v_black_up)
            black_up_[2] = config.v_black_up;
        if (motor_status_ != config.motor_status)
            motor_status_ = config.motor_status;
    }
} // namespace motion_controller
