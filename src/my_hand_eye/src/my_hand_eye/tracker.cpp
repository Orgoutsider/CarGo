#include "my_hand_eye/tracker.h"

namespace my_hand_eye
{
    Tracker::Tracker() : ranges_(histRange_){};

    void Tracker::get_center(double &u, double &v)
    {
        u = rect_.x + rect_.width / 2.;
        v = rect_.y + rect_.height / 2.;
    }

    bool Tracker::target_init(cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                              const int color, double &speed)
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
            Mat rectImg, targetImgHSV;
            rectImg = cv_image.image(rect_);
            imshow("target", rectImg);
            cvtColor(rectImg, targetImgHSV, COLOR_BGR2HSV);
            calcHist(&targetImgHSV, 2, channels_, Mat(), hist_, 1, &histSize_, &ranges_, true, false);
            normalize(hist_, hist_, 0, 255, NORM_MINMAX);
            double u, v;
            get_center(u, v);
            // int rows = cv_image.image.rows;
            // int cols = cv_image.image.cols;
            speed = -1;
            calculate_speed(cv_image, speed);
            return true;
        }
        return false;
    }

    bool Tracker::target_tracking(cv_bridge::CvImage &cv_image, double &speed)
    {
        using namespace cv;
        Mat imageHSV, targetImgHSV;
        Mat calcBackImage;
        cvtColor(cv_image.image, imageHSV, COLOR_BGR2HSV);
        calcBackProject(&imageHSV, 2, channels_, hist_, calcBackImage, &ranges_); // 反向投影
        TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001);
        CamShift(calcBackImage, rect_, criteria); // 关键函数
        // ROS_INFO("u:%lf v:%lf", u, v);
        speed = -1;
        bool valid = calculate_speed(cv_image, speed);
        Mat imageROI = imageHSV(rect_); // 更新模板
        targetImgHSV = imageHSV(rect_);
        calcHist(&imageROI, 2, channels_, Mat(), hist_, 1, &histSize_, &ranges_);
        normalize(hist_, hist_, 0.0, 1.0, NORM_MINMAX); // 归一化
        return valid;
    }

    bool Tracker::calculate_speed(cv_bridge::CvImage &cv_image, double &speed)
    {
        double u = 0, v = 0;
        get_center(u, v);
        if (!last_time_.is_zero() && !cv_image.header.stamp.is_zero() && last_pt_.x && last_pt_.y)
        {
            double du = u - last_pt_.x;
            double dv = v - last_pt_.y;
            ros::Duration dr = cv_image.header.stamp - last_time_;
            double dt = dr.toSec();
            speed = sqrt(du * du + dv * dv) / dt;
            last_time_ = cv_image.header.stamp;
            last_pt_ = cv::Point2d(u, v);
            return true;
        }
        else if (cv_image.header.stamp.is_zero())
        {
            ROS_WARN("Stamp is zero!");
            speed = -1;
            return false;
        }
        else
        {
            last_time_ = cv_image.header.stamp;
            last_pt_ = cv::Point2d(u, v);
            speed = -1;
            return false;
        }
    }
} // namespace my_hand_eye
