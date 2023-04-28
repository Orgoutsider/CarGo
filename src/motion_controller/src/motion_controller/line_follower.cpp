#include "motion_controller/line_follower.h"

namespace motion_controller
{
    LineFollower::LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : r_start_(110), r_end_(150),
          c_start_(10), c_end_(width_ - c_start_), threshold_(50),
          judge_line_(10), linear_velocity_(0.2),
          rho_thr_(5), theta_thr_(3),
          kp_(0.0015), kd_(0.001),
          black_low_(0, 0, 0), black_up_(180, 255, 100), motor_status_(false)
    {
        it_ = std::shared_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(nh));
        std::string transport_hint;
        pnh.param<std::string>("transport_hint", transport_hint, "raw");
        image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1, &LineFollower::_image_callback, this, image_transport::TransportHints(transport_hint));
        cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_line", 1);
        pnh.param<bool>("param_modification", param_modification_, false);
        if (param_modification_)
            server_.setCallback(boost::bind(&LineFollower::_dr_callback, this, _1, _2));
    }

    void LineFollower::_clean_lines(cv::Vec2f lines[], int num, double &rho_aver, double &theta_aver)
    {
        if (num == 0)
        {
            ROS_ERROR("num is zero!");
            return;
        }
        double rho_aver_ori = rho_aver;
        double theta_aver_ori = theta_aver;
        rho_aver *= num;
        theta_aver *= num;
        int num_ori = num;
        for (size_t i = 0; i < num_ori; i++)
        {
            double rho = lines[i][0];
            double theta = lines[i][1];
            if ((abs(rho_aver_ori - rho) > rho_thr_ ||
                 abs(theta_aver_ori - theta) > theta_thr_ / 180 * CV_PI) &&
                num > 1)
            {
                rho_aver -= rho;
                theta_aver -= theta;
                num--;
            }
        }
        theta_aver /= num;
        rho_aver /= num;
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
            cv_image = cv_bridge::toCvCopy(image_rect, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Mat srcF;
        resize(cv_image->image, srcF, Size(width_, height_));
        srcF = srcF(Range(r_start_, r_end_), Range(c_start_, c_end_));
        srcF.convertTo(srcF, CV_8UC3);
        // // 原先代码
        // // 色彩分离查找车道线
        // cvtColor(srcF, srcF, COLOR_BGR2HSV);
        // inRange(srcF, black_low_, black_up_, srcF);
        // if (param_modification_)
        // {
        //     imshow("inRange", srcF);
        //     waitKey(1);
        // }
        // // 可能需要去除干扰轮廓,(5, 3)可能更好一点
        // Mat element = getStructuringElement(MORPH_RECT, Size(7, 5));
        // // morphologyEx(srcF, srcF, MORPH_CLOSE, element);
        // dilate(srcF, srcF, element);
        // if (param_modification_)
        // {
        //     imshow("line1", srcF);
        //     waitKey(1);
        // }
        // Canny(srcF, srcF, 50, 100, 3);
        // if (param_modification_)
        // {
        //     imshow("line2", srcF);
        //     waitKey(1);
        // }
        // 分水岭代码
        Mat srcF_FenGe;
        cvtColor(srcF, srcF_FenGe, COLOR_BGR2GRAY);
        GaussianBlur(srcF_FenGe, srcF_FenGe, Size(3, 3), 0, 0);
        Canny(srcF_FenGe, srcF_FenGe, 50, 100, 3);

        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;
        findContours(srcF_FenGe, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
        Mat imageContours = Mat::zeros(srcF.size(), CV_8UC1); // 轮廓
        Mat marks(srcF.size(), CV_32S);
        marks = Scalar::all(0);
        int index = 0;
        int compCount = 0;
        // 这个循环在使用时注意一下，可能会出问题
        for (; index >= 0; index = hierarchy[index][0], compCount++)
        {
            drawContours(marks, contours, index, Scalar::all(compCount + 1), 1, 8, hierarchy);
            if (param_modification_)
                drawContours(imageContours, contours, index, Scalar(255), 1, 8, hierarchy);
        }
        // 中间图像，调试用
        if (param_modification_)
        {
            Mat marksShows;
            convertScaleAbs(marks, marksShows);
            imshow("marksShow", marksShows);
            imshow("Contours", imageContours);
            waitKey(1);
        }
        watershed(srcF, marks); // 分水岭检测

        // 中间图像，调试用
        if (param_modification_)
        {
            Mat afterWatershed;
            convertScaleAbs(marks, afterWatershed);
            imshow("After Watershed", afterWatershed);
        }

        // 对每一个区域进行颜色填充，调试用
        Mat PerspectiveImage = Mat::zeros(srcF.size(), CV_8UC3);
        for (int i = 0; i < marks.rows; i++)
        {
            for (int j = 0; j < marks.cols; j++)
            {
                int index = marks.at<int>(i, j);
                if (marks.at<int>(i, j) == -1)
                {
                    PerspectiveImage.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
                }
                else
                {
                    // 随机颜色生成
                    int value = index % 255;
                    RNG rng;
                    int aa = rng.uniform(0, value);
                    int bb = rng.uniform(0, value);
                    int cc = rng.uniform(0, value);
                    PerspectiveImage.at<Vec3b>(i, j) = Vec3b(aa, bb, cc);
                }
            }
        }
        imshow("After ColorFill", PerspectiveImage);

        // 分割并填充颜色的结果跟原始图像融合，调试用
        if (param_modification_)
        {
            Mat wshed;
            addWeighted(srcF, 0.4, PerspectiveImage, 0.6, 0, wshed);
            imshow("AddWeighted Image", wshed);
        }

        // 然后在这里接入轮廓的一些筛选和排除，然后进行直线识别
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, threshold_, 0, 0); // 根据实际调整，夜晚40，下午50
        int minx, maxx, nowx;
        static int last_err = 0, flag = 0;
        geometry_msgs::Twist twist;

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

        if (lines.size() > 1)
        {
            // 遍历直线求平均，分成左右两边
            for (Vec2f &line : lines)
            {
                double rho = line[0], theta = line[1];
                // if(abs(rho-rho_NUM)>rho_Derta && abs(theta-theta_NUM)>theta_Derta || abs(theta) < 3)//最后这个是滤去一些theta错误的直线，可以不加
                //     continue;
                if (theta > 0)
                {
                    rho_aver[left] += rho;
                    theta_aver[left] += theta;
                    lines_left_right[left][num[left]] = line;
                    num[left]++;
                }
                else if (theta < 0)
                {
                    rho_aver[right] += rho;
                    theta_aver[right] += theta;
                    lines_left_right[right][num[right]] = line;
                    num[right]++;
                }
                if (param_modification_)
                {
                    double a = cos(theta), b = sin(theta);
                    Mat res(srcF.size(), CV_8UC3, Scalar::all(0));
                    double x0 = a * rho, y0 = b * rho;
                    Point pt1, pt2;
                    pt1.x = cvRound(x0 + 1000 * (-b));
                    pt1.y = cvRound(y0 + 1000 * (a));
                    pt2.x = cvRound(x0 - 1000 * (-b));
                    pt2.y = cvRound(y0 - 1000 * (a));
                    cv::line(res, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                    imshow("HoughLines", res);
                    waitKey(1);
                }
            }
            if ((num[left] == 0) || (num[right] == 0))
                return;
            rho_aver[left] /= num[left];
            theta_aver[left] /= num[left];
            rho_aver[right] /= num[right];
            theta_aver[right] /= num[right];
            // 去除错误直线并拟合左右车道线
            _clean_lines(lines_left_right[left], num[left], rho_aver[left], theta_aver[left]);
            _clean_lines(lines_left_right[right], num[right], rho_aver[right], theta_aver[right]);

            for (int i = left; i <= right; i++)
            {
                double a = cos(theta_aver[i]), b = sin(theta_aver[i]);
                x[i] = ((rho_aver[i] - b * judge_line_) / a);
                // 直线绘制
                if (param_modification_)
                {
                    Mat res(srcF.size(), CV_8UC3, Scalar::all(0));
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
            double err = (x[left] + x[right]) / 2 - srcF.cols / 2;
            // 滤除不正常的误差
            err = (abs(err) > (c_end_ - c_start_) / 7.0) ? last_err : err;
            if (flag = 0)
            {
                flag = 1;
                last_err = err;
            }
            double d_err = err - last_err;
            twist.linear.x = linear_velocity_;
            twist.angular.z = (err != 0) ? -kp_ * err - kd_ * d_err : 0;
            last_err = err;
            // ROS_INFO_STREAM("err" << err << " size:" << lines.size());
            // ROS_INFO_STREAM(minx << ' ' << maxx);
        }
        else
        {
            // 即使没有车道线的参照，仍然前进
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
                ROS_WARN("r_end must be greater than r_start!");
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
        if (rho_thr_ != config.rho_thr)
            rho_thr_ = config.rho_thr;
        if (theta_thr_ != config.theta_thr)
            theta_thr_ = config.theta_thr;
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
