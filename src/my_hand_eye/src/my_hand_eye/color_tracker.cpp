#include <boost/thread/mutex.hpp>

#include "my_hand_eye/color_tracker.h"

namespace my_hand_eye
{

    Angle ColorMethod::hue_value(double h_val)
    {
        return Angle(h_val * 2);
    }

    double ColorMethod::hue_value_tan(double y, double x)
    {
        return Angle::atan2(y, x).get_degree() / 2;
    }

    double ColorMethod::hue_value_aver(cv::Mat &&roi, int white_vmin)
    {
        cv::Mat mask;
        roi.convertTo(mask, CV_8UC3);
        cv::cvtColor(mask, mask, cv::COLOR_BGR2HSV);
        double x = 0, y = 0;
        int cnt = 0;
        boost::mutex mtx;
        mask.forEach<cv::Vec3b>([&](cv::Vec3b &pixel, const int position[])
                                {
            if (pixel[1] <= white_smax_ && pixel[2] >= white_vmin)
            {
                // pixel[0] = pixel[1] = pixel[2] = 0;
                return;
            }
            Angle color = hue_value(pixel[0]);
            mtx.lock();
            x += color.cos();
            y += color.sin();
            cnt++;
            mtx.unlock(); });
        // if (!mask.empty())
        // {
        //     cv::imshow("mask_img", mask); // 用于调试
        //     cv::waitKey(10);
        // }
        if (cnt > 0)
        {
            double H_Average = hue_value_tan(y, x); // 保存当前区域色相H的平均值
            return (H_Average < 0) ? H_Average + 180 : H_Average;
        }
        else
            return -1;
    }

    double ColorMethod::hue_value_diff(double h_val1, double h_val2)
    {
        Angle a1 = hue_value(h_val1);
        Angle a2 = hue_value(h_val2);
        double res = abs((a1 - a2).get_degree());
        if (res > 180)
        {
            res = 360.0 - res;
        }
        return res / 2;
    }

    ColorTracker::ColorTracker() : gain_(0.4), speed_max_(1.7),
                                   s_min_{43, 43, 43, 43},
                                   left_color(0), right_color(0) {}

    bool ColorTracker::_set_rect(cv_bridge::CvImage &cv_image, cv::Rect &roi)
    {
        using namespace cv;
        Mat hsv = cv_image.image(roi).clone();
        GaussianBlur(hsv, hsv, Size(3, 3), 0, 0);
        cvtColor(hsv, hsv, COLOR_BGR2HSV);
        Mat dst, white;
        if (h_max[color_] >= h_min[color_])
        {
            Scalar low = Scalar(h_min[color_], s_min_[color_], v_min[color_]);
            Scalar up = Scalar(h_max[color_], 255, 255);
            inRange(hsv, low, up, dst);
        }
        else // 色相范围分成两段的情况，如红色
        {
            Scalar low1 = Scalar(0, s_min_[0], v_min[0]);
            Scalar up1 = Scalar(h_max[color_], 255, 255);
            Scalar low2 = Scalar(h_min[color_], s_min_[color_], v_min[color_]);
            Scalar up2 = Scalar(180, 255, 255);
            Mat dst1, dst2;
            inRange(hsv, low1, up1, dst1);
            inRange(hsv, low2, up2, dst2);
            bitwise_or(dst1, dst2, dst);
        }

        // 白色
        Scalar l = Scalar(0, 0, white_vmin_);
        Scalar u = Scalar(180, white_smax_, 255);
        inRange(hsv, l, u, white);
        bitwise_not(white, white);
        bitwise_and(dst, white, dst);
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(13, 13));
        erode(dst, dst, element);
        // if (show_detections_)
        // {
        //     imshow("dst", dst); // 用于调试
        //     waitKey(1);
        // }
        std::vector<std::vector<cv::Point>> contours;                  // 轮廓容器
        Point2f vtx[4];                                                // 矩形顶点容器
        findContours(dst, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE); // 查找轮廓
        Mat res = Mat::zeros(dst.size(), CV_8UC3);                     // 创建空白图像，用于调试
        double s_max = 0;                                              // 最大面积
        int i_max = 0;                                                 // 最大面积对应索引
        for (int i = 0; i < contours.size(); i++)
        {
            RotatedRect rect = minAreaRect(contours[i]); // 求解最小矩阵
            if (rect.size.area() > s_max)
            {
                s_max = rect.size.area();
                i_max = i;
                rect_ = rect;
            }
        }
        if (s_max)
        {
            if (show_detections_)
            {
                Scalar colors;
                switch (color_)
                {
                case color_red:
                    colors = Scalar(0, 0, 255);
                    break;
                case color_green:
                    colors = Scalar(0, 255, 0);
                    break;
                case color_blue:
                    colors = Scalar(255, 0, 0);
                    break;
                default:
                    ROS_ERROR("Color error!");
                    return false;
                }
                drawContours(res, contours, i_max, colors, 1); // 随机颜色绘制轮廓
                rect_.points(vtx);                             // 确定旋转矩阵的四个顶点
                for (int j = 0; j < 4; j++)
                {
                    line(res, vtx[j], vtx[(j + 1) % 4], colors, 2);
                }
                // imshow("res", res); // 用于调试
                // waitKey(1);
            }
            rect_.center.x += roi.x;
            rect_.center.y += roi.y;
            return true;
        }
        else
        {
            ROS_WARN("_set_rect: contours are empty!");
            // imshow("dst", dst); // 用于调试
            // imshow("src", hsv);
            // waitKey(10);
            return false;
        }
    }

    cv::Rect ColorTracker::_rect_in_image(cv_bridge::CvImage &cv_image, int c_start, int r_start,
                                          int c_end, int r_end)
    {
        cv::Rect rect_new;
        rect_new.x = MAX(0, c_start);
        rect_new.y = MAX(0, r_start);
        rect_new.width = MAX(0, MIN(cv_image.image.cols, c_end) - rect_new.x);
        rect_new.height = MAX(0, MIN(cv_image.image.rows, r_end) - rect_new.y);
        return rect_new;
    }

    cv::Rect ColorTracker::_predict_rect(cv_bridge::CvImage &cv_image, Pos &ps, cv::Rect rect_ori,
                                         double this_theta, double z, bool read)
    {
        this_theta = (this_theta > CV_PI)
                         ? this_theta - 2 * CV_PI
                         : (this_theta <= -CV_PI ? this_theta + 2 * CV_PI : this_theta);
        double this_x = center_x_ + cos(this_theta) * radius_;
        double this_y = center_y_ + sin(this_theta) * radius_;
        double u = 0, v = 0;
        ps.calculate_pixel_position(this_x, this_y, z, u, v, read);
        return _rect_in_image(cv_image, u - rect_ori.width / 2, v - rect_ori.height / 2,
                              u + rect_ori.width / 2, v + rect_ori.height / 2);
    }

    bool ColorTracker::_update_time(cv_bridge::CvImage &cv_image)
    {
        if (!this_time_.is_zero() && !cv_image.header.stamp.is_zero())
        {
            last_time_ = this_time_;
            this_time_ = cv_image.header.stamp;
            return true;
        }
        else if (cv_image.header.stamp.is_zero())
        {
            ROS_WARN("Stamp is zero!");
            last_time_ = this_time_;
            this_time_ = ros::Time();
            return false;
        }
        else
        {
            this_time_ = cv_image.header.stamp;
            return true;
        }
    }

    void ColorTracker::get_center(double &u, double &v)
    {
        u = rect_.center.x;
        v = rect_.center.y;
    }

    bool ColorTracker::target_init(ros::NodeHandle &nh,
                                   cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                                   const Color color, int white_vmin, double center_x, double center_y,
                                   bool show_detections)
    {
        using namespace cv;
        if (objArray.boxes.size() == 4)
        {
            if (h_max.empty() || h_min.empty() || v_min.empty())
            {
                if (nh.hasParam("h_max"))
                    nh.getParam("h_max", h_max);
                else
                {
                    ROS_WARN("Parameter h_max not available.");
                    return false;
                }
                if (nh.hasParam("h_min"))
                    nh.getParam("h_min", h_min);
                else
                {
                    ROS_WARN("Parameter h_min not available.");
                    return false;
                }
                if (nh.hasParam("v_min"))
                    nh.getParam("v_min", v_min);
                else
                {
                    ROS_WARN("Parameter v_min not available.");
                    return false;
                }
            }
            color_ = color;
            show_detections_ = show_detections;
            if (!objArray.boxes[color_].center.x)
                return false;
            double width = objArray.boxes[color_].size_x;
            double height = objArray.boxes[color_].size_y;
            // 把目标物体的矩形框传递进来
            Rect rect_ori(objArray.boxes[color_].center.x - width / 2,
                          objArray.boxes[color_].center.y - height / 2, width, height);
            last_time_ = this_time_ = ros::Time();
            _update_time(cv_image);
            last_pt_.x = last_pt_.y = 0;
            center_x_ = center_x;
            center_y_ = center_y;
            white_vmin_ = white_vmin;
            // if (show_detections_)
            // {
            //     imshow("ori", cv_image.image(rect_ori));
            //     waitKey(3);
            // }
            return _set_rect(cv_image, rect_ori);
        }
        return false;
    }

    bool ColorTracker::order_init(vision_msgs::BoundingBox2DArray &objArray, Pos &ps, double z)
    {
        if (objArray.boxes.size() == 4)
        {
            int c[2] = {0};
            double theta[2] = {0};
            double theta_n = 0;
            int cnt = 0;
            for (int color = color_red; color <= color_blue; color++)
            {
                if (color == color_)
                {
                    if (!objArray.boxes[color].center.x)
                        return false;
                    double u = objArray.boxes[color].center.y;
                    double v = objArray.boxes[color].center.x;
                    double x = 0, y = 0;
                    if (ps.calculate_cargo_position(u, v, z, x, y, false))
                    {
                        theta_n = atan2(y - center_y_, x - center_x_);
                    }
                    else
                        return false;
                    continue;
                }
                // 不是正要抓的
                if (!objArray.boxes[color].center.x)
                    return false;
                else
                {
                    double u = objArray.boxes[color].center.y;
                    double v = objArray.boxes[color].center.x;
                    double x = 0, y = 0;
                    if (ps.calculate_cargo_position(u, v, z, x, y, false))
                    {
                        c[cnt] = color;
                        theta[cnt++] = atan2(y - center_y_, x - center_x_);
                    }
                    else
                        return false;
                }
            }
            for (int i = 0; i <= 1; i++)
                theta[i] = theta[i] > theta_n ? theta[i] : theta[i] + 2 * CV_PI;
            if (theta[0] > theta[1])
            {
                right_color = c[1];
                left_color = c[0];
            }
            else
            {
                right_color = c[0];
                left_color = c[1];
            }
            return true;
        }
        else
            return false;
    }

    bool ColorTracker::target_track(cv_bridge::CvImage &cv_image, Pos &ps, double z)
    {
        using namespace cv;
        Rect2f rect = rect_.boundingRect2f();
        Rect rect_ori = _rect_in_image(cv_image, rect.x - rect.width * gain_,
                                       rect.y - rect.height * gain_,
                                       rect.x + rect.width + rect.width * gain_,
                                       rect.y + rect.height + rect.height * gain_);
        // ROS_INFO_STREAM("rect_ori.x:" << rect_ori.x << " rect_ori.y:" << rect_ori.y << " rect.x:" << rect.x << " rect.y:" << rect.y);
        // 下面计算物料下一个可能位置范围
        bool valid = _update_time(cv_image);
        if (valid)
        {
            double last_theta = atan2(last_pt_.y - center_y_, last_pt_.x - center_x_);
            double dt = (this_time_ - last_time_).toSec();
            double d_theta = (flag ? 1 : -1) * dt * speed_max_;
            Rect rect_new = rect_ori;
            // ROS_INFO_STREAM("last_theta:" << last_theta << " center_x:" << center_x_ << " center_y:" << center_y_ << " x:" << last_pt_.x << " y:" << last_pt_.y);
            rect_new |= _predict_rect(cv_image, ps, rect_ori, last_theta + 1.0 / 6 * d_theta, z, true);
            rect_new |= _predict_rect(cv_image, ps, rect_ori, last_theta + 2.0 / 6 * d_theta, z, false);
            rect_new |= _predict_rect(cv_image, ps, rect_ori, last_theta + 3.0 / 6 * d_theta, z, false);
            rect_new |= _predict_rect(cv_image, ps, rect_ori, last_theta + 4.0 / 6 * d_theta, z, false);
            rect_new |= _predict_rect(cv_image, ps, rect_ori, last_theta + 5.0 / 6 * d_theta, z, false);
            rect_new |= _predict_rect(cv_image, ps, rect_ori, last_theta + d_theta, z, false);
            // if (show_detections_)
            // {
            //     imshow("new", cv_image.image(rect_new));
            //     waitKey(3);
            // }

            // 生成新位置
            valid = _set_rect(cv_image, rect_ori);
        }
        return valid;
    }

    bool ColorTracker::calculate_radius_and_speed(double x, double y, double &radius,
                                                  double speed_standard_static, double &speed)
    {
        radius_ = sqrt((x - center_x_) * (x - center_x_) + (y - center_y_) * (y - center_y_));
        radius = radius_;
        static int err_cnt = 0;
        if (!last_time_.is_zero() && !this_time_.is_zero() && last_pt_.x && last_pt_.y)
        {
            double this_theta = atan2(y - center_y_, x - center_x_);
            double last_theta = atan2(last_pt_.y - center_y_, last_pt_.x - center_x_);
            double dt = (this_time_ - last_time_).toSec();
            double d_theta[2];
            d_theta[0] = (this_theta < last_theta) ? last_theta - this_theta : last_theta + CV_PI * 2 - this_theta;
            d_theta[1] = (this_theta >= last_theta) ? this_theta - last_theta : this_theta + CV_PI * 2 - last_theta;
            last_pt_ = cv::Point2d(x, y);
            // ROS_INFO_STREAM("last_theta:" << last_theta << " center_x:" << center_x_ << " center_y:" << center_y_ << " x:" << last_pt_.x << " y:" << last_pt_.y);
            if (d_theta[flag] <= d_theta[!flag] || d_theta[!flag] == 0)
            {
                if (d_theta[!flag] == 0)
                    speed = 0;
                else
                    speed = d_theta[flag] / dt;
                if (err_cnt)
                    err_cnt = 0;
                return true;
            }
            else
            {
                speed = 0;
                if (speed < speed_standard_static)
                {
                    if (err_cnt)
                        err_cnt = 0;
                    return true;
                }
                else if (err_cnt > 6)
                {
                    err_cnt = 0;
                    ROS_WARN_ONCE("Direction may not be right!");
                    return true;
                }
                else
                {
                    err_cnt++;
                    return true;
                }
            }
        }
        else if (this_time_.is_zero())
        {
            speed = -1;
            ROS_WARN("this_time_ is zero!");
            return false;
        }
        else // 初始化
        {
            speed = -1;
            last_pt_ = cv::Point2d(x, y);
            return false;
        }
    }

    bool ColorTracker::no_obstacles()
    {
        double theta = atan2(last_pt_.y - center_y_, last_pt_.x - center_x_); // -pi~pi
        theta = (theta > CV_PI / 2.0) ? theta - 2 * CV_PI : theta;            // 限制在-pi/2周围
        static ros::Time stop;
        if (!stop.is_zero() && (ros::Time::now() - stop).toSec() < 4)
            return false;
        if (abs(theta + CV_PI / 2) > (CV_PI * 0.38))
        {
            stop = ros::Time::now();
            return false; // 放弃太远的物料
        }
        else
            ROS_INFO_STREAM(abs(theta + CV_PI / 2) / CV_PI << " pi of theta is ok to catch.");
        if (left_color)
        {
            double t = theta - 2.0 / 3 * CV_PI;
            t = (t <= -CV_PI * 3 / 2.0) ? t + 2 * CV_PI : t;
            // ROS_INFO_STREAM("theta " << theta << " " << t);
            // ROS_INFO_STREAM(abs(t + CV_PI / 2) << " " << abs(theta + CV_PI / 2));
            if (abs(t + CV_PI / 2) + CV_PI / 3 < abs(theta + CV_PI / 2))
            {
                stop = ros::Time::now();
                return false;
            }
        }
        if (right_color)
        {
            double t = theta + 2.0 / 3 * CV_PI;
            t = (t > CV_PI / 2.0) ? t - 2 * CV_PI : t;
            // ROS_INFO_STREAM("theta " << theta << " " << t);
            // ROS_INFO_STREAM(abs(t + CV_PI / 2) << " " << abs(theta + CV_PI / 2));
            if (abs(t + CV_PI / 2) + CV_PI / 5 < abs(theta + CV_PI / 2))
            {
                stop = ros::Time::now();
                return false;
            }
        }
        return true;
    }
} // namespace my_hand_eye
