#include "my_hand_eye/color_tracker.h"

namespace my_hand_eye
{

    Angle ColorMethod::hue_value(double h_val)
    {
        return Angle(h_val * 2);
    }

    double ColorMethod::hue_value_tan(double y, double x)
    {
        return Angle(y, x)._get_degree() / 2;
    }

    double ColorMethod::hue_value_aver(cv::Mat &&roi, int white_vmin)
    {
        cv::Mat mask = roi.clone();
        cv::Mat mask_Img = mask.clone();        // 用于调试
        mask_Img = {cv::Scalar(255, 255, 255)}; // 用于调试
        cvtColor(mask, mask, cv::COLOR_BGR2HSV);
        // 设置像素遍历迭代器
        cv::MatConstIterator_<cv::Vec3b> maskStart = mask.begin<cv::Vec3b>();
        cv::MatConstIterator_<cv::Vec3b> maskEnd = mask.end<cv::Vec3b>();
        cv::MatIterator_<cv::Vec3b> mask_ImgStart = mask_Img.begin<cv::Vec3b>(); // 用于调试
        double x = 0, y = 0;
        int cnt = 0;
        for (; maskStart != maskEnd; maskStart++, mask_ImgStart++)
        {
            // 过滤白色
            if ((*maskStart)[1] <= white_smax_ && (*maskStart)[2] >= white_vmin)
            {
                // 用于调试
                (*mask_ImgStart)[0] = 0;
                (*mask_ImgStart)[1] = 0;
                (*mask_ImgStart)[2] = 0;
                continue;
            }
            int H_Val = (*maskStart)[0];
            Angle color = hue_value(H_Val);
            x += color.cos();
            y += color.sin();
            cnt++;
            // if (cnt % 200 == 0)
            // {
            //     ROS_INFO_STREAM(H_Val);
            //     usleep(1e5);
            // }
        }
        cv::imshow("mask_Img", mask_Img); // 用于调试
        cv::waitKey(10);
        double H_Average = hue_value_tan(y, x); // 保存当前区域色相H的平均值
        return (H_Average < 0) ? H_Average + 180 : H_Average;
    }

    double ColorMethod::hue_value_diff(double h_val1, double h_val2)
    {
        Angle a1 = hue_value(h_val1);
        Angle a2 = hue_value(h_val2);
        double res = abs((a1 - a2)._get_degree());
        if (res > 180)
        {
            res = 360.0 - res;
        }
        return res / 2;
    }

    ColorTracker::ColorTracker() : channels_{0}, histSize_{180},
                                   pranges_{0, 179}, ranges_{pranges_},
                                   gain_(0.1), speed_max_(0.7),
                                   s_min_(43), v_min_(26),
                                   flag_(false) {}

    bool ColorTracker::_set_rect(cv::Mat &hsv)
    {
        using namespace cv;
        Mat dst;
        if (h_max_ >= h_min_)
        {
            Scalar low = Scalar(h_min_, s_min_, v_min_);
            Scalar up = Scalar(h_max_, 255, 255);
            inRange(hsv, low, up, dst);
        }
        else // 色相范围分成两段的情况，如红色
        {
            Scalar low1 = Scalar(0, s_min_, v_min_);
            Scalar up1 = Scalar(h_max_, 255, 255);
            Scalar low2 = Scalar(h_min_, s_min_, v_min_);
            Scalar up2 = Scalar(180, 255, 255);
            Mat dst1, dst2;
            inRange(hsv, low1, up1, dst1);
            inRange(hsv, low2, up2, dst2);
            bitwise_or(dst1, dst2, dst);
        }
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(13, 13));
        erode(dst, dst, element);
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
            imshow("res", res); // 用于调试
            waitKey(1);
            return true;
        }
        else
        {
            ROS_ERROR("_set_rect: contours are empty!");
            return false;
        }
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

    bool ColorTracker::target_init(cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                                   const int color, int white_vmin, double center_x, double center_y,
                                   double proportion)
    {
        using namespace cv;
        if (objArray.boxes.size() == 4)
        {
            color_ = color;
            if (!objArray.boxes[color_].center.x)
                return false;
            double width = objArray.boxes[color_].size_x;
            double height = objArray.boxes[color_].size_y;
            // 把目标物体的矩形框传递进来
            Rect rect_ori(objArray.boxes[color_].center.x - width / 2,
                          objArray.boxes[color_].center.y - height / 2, width, height);
            _update_time(cv_image);
            last_pt_.x = last_pt_.y = 0;
            center_x_ = center_x;
            center_y_ = center_y;

            Mat HSVImg = cv_image.image(rect_ori).clone();
            GaussianBlur(HSVImg, HSVImg, Size(3, 3), 0, 0);
            cvtColor(HSVImg, HSVImg, COLOR_BGR2HSV);
            std::vector<Mat> HSV_planes;
            split(HSVImg, HSV_planes);

            // Mat H_planes(HSV_planes[0], CV_32FC1);
            MatND hist;
            calcHist(&HSV_planes[0], 1, channels_, Mat(), hist, 1, histSize_, ranges_);
            normalize(hist, hist, 1.0, 0, NORM_L1, -1, Mat());
            int H_Average = cvRound(hue_value_aver(cv_image.image(rect_ori), white_vmin)); // 保存当前区域色相H的平均值
            // 色彩平均值的对侧
            int H_Opposite = (H_Average - 90 < 0) ? H_Average + 90 : H_Average - 90;
            int left, right;
            left = right = H_Opposite;
            bool fin_l = false, fin_r = false;
            double sum_l = hist.at<double>(H_Opposite), sum_r = hist.at<double>(H_Opposite);
            while (left != H_Average && right != H_Average)
            {
                if (!fin_l)
                {
                    left--;
                    if (left < 0)
                        left = 179;
                    sum_l += hist.at<double>(left);
                    if (sum_l > proportion / 2)
                        fin_l = true;
                }
                if (!fin_r)
                {
                    right++;
                    if (right > 179)
                        right = 0;
                    sum_r += hist.at<double>(right);
                    if (sum_r > proportion / 2)
                        fin_r = true;
                }
                if (fin_l && fin_r)
                    break;
            }
            if (fin_l && fin_r)
            {
                h_max_ = left;
                h_min_ = right;
            }
            else
            {
                ROS_ERROR("target_init: Proportion is invalid!");
                return false;
            }
            return _set_rect(HSVImg);
        }
        return false;
    }

    bool ColorTracker::target_track(cv_bridge::CvImage &cv_image, Pos &ps, double z)
    {
        using namespace cv;
        Rect2f rect = rect_.boundingRect2f();
        Rect rect_ori;
        rect_ori.x = std::max(0.0, rect.x - rect.width * gain_);
        rect_ori.y = std::max(0.0, rect.y - rect.height * gain_);
        rect_ori.width = std::min((double)cv_image.image.cols,
                                  rect.x + rect.width + rect.width * gain_) -
                         rect_ori.x;
        rect_ori.height = std::min((double)cv_image.image.rows,
                                   rect.y + rect.height + rect.height * gain_) -
                          rect_ori.y;

        // 下面计算物料下一个可能位置范围
        bool valid = _update_time(cv_image);
        if (valid)
        {
            double last_theta = atan2(last_pt_.y - center_y_, last_pt_.x - center_x_);
            double dt = (this_time_ - last_time_).toSec();
            double this_theta = last_theta + (flag_ ? 1 : -1) * dt;
            this_theta = (this_theta > CV_PI)
                             ? this_theta - 2 * CV_PI
                             : (this_theta <= -CV_PI ? this_theta + 2 * CV_PI : this_theta);
            double this_x = center_x_ + cos(this_theta);
            double this_y = center_y_ + sin(this_theta);
            double u, v;
            ps.calculate_pixel_position(this_x, this_y, z, u, v);

            Rect rect_new;
            rect_new.x = std::max(0.0, u - rect_ori.width / 2);
            rect_new.y = std::max(0.0, v - rect_ori.height / 2);
            rect_new.width = std::min((double)cv_image.image.cols,
                                      u + rect_ori.width / 2) -
                             rect_new.x;
            rect_new.height = std::min((double)cv_image.image.rows,
                                       v - rect_ori.height / 2) -
                              rect_new.y;
            rect_ori |= rect_new;

            // 生成新位置
            Mat HSVImg = cv_image.image(rect_ori).clone();
            GaussianBlur(HSVImg, HSVImg, Size(3, 3), 0, 0);
            cvtColor(HSVImg, HSVImg, COLOR_BGR2HSV);
            valid = _set_rect(HSVImg);
        }
        return valid;
    }

    bool ColorTracker::calculate_speed(double x, double y,
                                       double speed_standard, double &speed)
    {
        if (!last_time_.is_zero() && !this_time_.is_zero() && last_pt_.x && last_pt_.y)
        {
            double this_theta = atan2(y - center_y_, x - center_x_);
            double last_theta = atan2(last_pt_.y - center_y_, last_pt_.x - center_x_);
            double dt = (this_time_ - last_time_).toSec();
            double d_theta[2];
            d_theta[0] = (this_theta < last_theta) ? last_theta - this_theta : last_theta + CV_PI * 2 - this_theta;
            d_theta[1] = (this_theta > last_theta) ? this_theta - last_theta : this_theta + CV_PI * 2 - last_theta;
            last_pt_ = cv::Point2d(x, y);
            if (d_theta[flag_] <= d_theta[!flag_])
            {
                speed = d_theta[flag_] / dt;
                return true;
            }
            else if (speed < speed_standard)
            {
                flag_ = !flag_;
                speed = d_theta[flag_] / dt;
                return true;
            }
            else
            {
                flag_ = !flag_;
                speed = -1;
                ROS_WARN("Direction has changed!");
                return false;
            }
        }
        else if (this_time_.is_zero())
        {
            speed = -1;
            ROS_WARN("this_time_ is zero!");
            return false;
        }
        else
        {
            speed = -1;
            last_pt_ = cv::Point2d(x, y);
            return false;
        }
    }

} // namespace my_hand_eye
