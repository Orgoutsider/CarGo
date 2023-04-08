#include "my_hand_eye/tracker.h"

namespace my_hand_eye
{
    Tracker::Tracker() : ranges_(histRange_), histSize_(200){};

    void Tracker::get_center(double &u, double &v)
    {
        u = rect_.x + rect_.width / 2.;
        v = rect_.y + rect_.height / 2.;
    }

    bool Tracker::target_init(cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                              const int color, int method)
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
            if (method == tracker_camshift)
            {
                method_ = method;
                Mat rectImg, targetImgHSV;
                rectImg = cv_image.image(rect_);
                // imshow("target", rectImg);
                cvtColor(rectImg, targetImgHSV, COLOR_BGR2HSV);
                calcHist(&targetImgHSV, 2, channels_, Mat(), hist_, 1, &histSize_, &ranges_, true, false);
                normalize(hist_, hist_, 0, 255, NORM_MINMAX);
                _update_time(cv_image);
                return true;
                last_pt_.x = last_pt_.y = 0;
            }
            else
            {
                _create_tracker_by_method(method);
                bool ok = tracker_ptr_->init(cv_image.image, rect_);
                if (ok)
                {
                    _update_time(cv_image);
                    last_pt_.x = last_pt_.y = 0;
                }
                return ok;
            }
            // int rows = cv_image.image.rows;
            // int cols = cv_image.image.cols;
        }
        return false;
    }

    bool Tracker::target_tracking(cv_bridge::CvImage &cv_image)
    {
        using namespace cv;
        if (method_ == tracker_camshift)
        {
            Mat imageHSV, targetImgHSV;
            Mat calcBackImage;
            cvtColor(cv_image.image, imageHSV, COLOR_BGR2HSV);
            calcBackProject(&imageHSV, 2, channels_, hist_, calcBackImage, &ranges_); // 反向投影
            TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001);
            cv::Rect rect = rect_;
            CamShift(calcBackImage, rect, criteria); // 关键函数
            rect_ = rect;
            Mat imageROI = imageHSV(rect_); // 更新模板
            targetImgHSV = imageHSV(rect_);
            calcHist(&imageROI, 2, channels_, Mat(), hist_, 1, &histSize_, &ranges_);
            normalize(hist_, hist_, 0.0, 1.0, NORM_MINMAX); // 归一化
            // ROS_INFO("u:%lf v:%lf", u, v);
            bool valid = _update_time(cv_image);
            return valid;
        }
        else if (method_ == tracker_CRST || method_ == tracker_KCF || method_ == tracker_MOSSE)
        {
            bool valid = tracker_ptr_->update(cv_image.image, rect_);
            if (valid)
                valid = _update_time(cv_image);
            return valid;
        }
        ROS_ERROR("Incorrect track method!");
        return false;
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

    bool Tracker::_create_tracker_by_method(int method)
    {
        switch (method)
        {
        case tracker_CRST:
            tracker_ptr_ = cv::TrackerCSRT::create();
            break;
        case tracker_KCF:
            tracker_ptr_ = cv::TrackerKCF::create();
            break;
        case tracker_MOSSE:
            tracker_ptr_ = cv::TrackerMOSSE::create();
            break;
        default:
            ROS_ERROR("Incorrect track method!");
            return false;
        }
        method_ = method;
        return true;
    }

    MultiTracker::MultiTracker() {};

    bool MultiTracker::init(cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                            const int color, int method)
    {
        using namespace cv;
        color_now_ = color;
        if (objArray.boxes.size() == 4)
        {
            multi_tracker_ptr_ = cv::MultiTracker::create();
            for (int color = color_red; color <= color_blue; color++)
            {
                if (!objArray.boxes[color].center.x)
                    return false;
                double width = objArray.boxes[color].size_x;
                double height = objArray.boxes[color].size_y;
                Rect2d rect(objArray.boxes[color].center.x - width / 2,
                            objArray.boxes[color].center.y - height / 2, width, height);
                _create_tracker_by_method(method);
                multi_tracker_ptr_->add(tracker_ptr_, cv_image.image, rect);
                if (color == color_now_)
                    rect_ = rect;
            }
            last_pt_arr_.fill(cv::Point2d());
            _update_time(cv_image);
            return true;
        }
        return false;
    }

    void MultiTracker::tracking(cv_bridge::CvImage &cv_image)
    {
        // 用新帧更新跟踪结果
        multi_tracker_ptr_->update(cv_image.image);
        _update_time(cv_image);
        rect_ = multi_tracker_ptr_->getObjects()[color_now_ - 1];
    }
    
    bool MultiTracker::speed(double x[4], double y[4], double center_x, double center_y,
                             double speed_standard, double speed[4])
    {
        for (int color = color_red; color <= color_blue; color++)
        {
            last_pt_ = last_pt_arr_[color];
            if (calculate_speed(x[color], y[color], center_x, center_y, speed_standard, speed[color]))
                return false;
            last_pt_arr_[color] = last_pt_;
        }
        return true;
    }
} // namespace my_hand_eye
