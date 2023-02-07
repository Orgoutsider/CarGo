#include "my_hand_eye/tracker.h"

namespace my_hand_eye
{
    Tracker::Tracker(){};

    void Tracker::get_center(double &u, double &v)
    {
        u = rect_.x + rect_.width / 2.;
        v = rect_.y + rect_.height / 2.;
    }

    bool Tracker::target_init(cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                              const int color)
    {
        using namespace cv;
        if (objArray.boxes.size() == 4)
        {
            if (!objArray.boxes[color].center.x)
                return false;
            double width = objArray.boxes[color].size_x;
            double height = objArray.boxes[color].size_y;
            // CamShift算法要求要把目标物体的矩形框传递进来
            rect_ = Rect(objArray.boxes[color].center.x - width / 2,
                         objArray.boxes[color].center.y - height / 2, width, height);
            tracker_ptr_ = cv::TrackerCSRT::create();
            bool ok = tracker_ptr_->init(cv_image.image, rect_);
            if (ok)
                _update_time(cv_image);
            return ok;
            // Mat rectImg, targetImgHSV;
            // rectImg = cv_image.image(rect_);
            // imshow("target", rectImg);
            // cvtColor(rectImg, targetImgHSV, COLOR_BGR2HSV);
            // calcHist(&targetImgHSV, 2, channels_, Mat(), hist_, 1, &histSize_, &ranges_, true, false);
            // normalize(hist_, hist_, 0, 255, NORM_MINMAX);
            // int rows = cv_image.image.rows;
            // int cols = cv_image.image.cols;
        }
        return false;
    }

    bool Tracker::target_tracking(cv_bridge::CvImage &cv_image)
    {
        using namespace cv;
        bool valid = tracker_ptr_->update(cv_image.image, rect_);
        if (valid)
            valid = _update_time(cv_image);
        // Mat imageHSV, targetImgHSV;
        // Mat calcBackImage;
        // cvtColor(cv_image.image, imageHSV, COLOR_BGR2HSV);
        // calcBackProject(&imageHSV, 2, channels_, hist_, calcBackImage, &ranges_); // 反向投影
        // TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001);
        // CamShift(calcBackImage, rect_, criteria); // 关键函数
        // Mat imageROI = imageHSV(rect_); // 更新模板
        // targetImgHSV = imageHSV(rect_);
        // calcHist(&imageROI, 2, channels_, Mat(), hist_, 1, &histSize_, &ranges_);
        // normalize(hist_, hist_, 0.0, 1.0, NORM_MINMAX); // 归一化
        // ROS_INFO("u:%lf v:%lf", u, v);
        return valid;
    }

    bool Tracker::_update_time(cv_bridge::CvImage &cv_image)
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

    bool Tracker::calculate_speed(double x, double y, double center_x, double center_y,
                                  double speed_standard, double &speed)
    {
        if (!last_time_.is_zero() && !this_time_.is_zero() && last_pt_.x && last_pt_.y)
        {
            double this_theta = atan2(y - center_y, x - center_x);
            double last_theta = atan2(last_pt_.y - center_y, last_pt_.x - center_x);
            double dt = (this_time_ - last_time_).toSec();
            static bool flag = false; // 顺/逆时针标志
            double d_theta[2];
            d_theta[0] = (this_theta < last_theta) ? last_theta - this_theta : last_theta + CV_PI * 2 - this_theta;
            d_theta[1] = (this_theta > last_theta) ? this_theta - last_theta : this_theta + CV_PI * 2 - last_theta;
            last_pt_ = cv::Point2d(x, y);
            if (d_theta[flag] <= d_theta[!flag])
            {
                speed = d_theta[flag] / dt;
                return true;
            }
            else if (speed < speed_standard)
            {
                flag = !flag;
                speed = d_theta[flag] / dt;
                return true;
            }
            else
            {
                flag = !flag;
                speed = -1;
                ROS_WARN("Direction changed!");
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
