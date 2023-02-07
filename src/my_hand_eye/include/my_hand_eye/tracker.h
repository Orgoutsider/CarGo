#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "opencv2/opencv.hpp"
#include "opencv2/tracking/tracker.hpp"

#include <ros/ros.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

namespace my_hand_eye
{
    class Tracker
    {
    private:
        // 椭圆圆心坐标
        cv::Point2d last_pt_;
        cv::Ptr<cv::Tracker> tracker_ptr_;
        ros::Time last_time_;
        ros::Time this_time_;
        // cv::Mat hist_;
        // const int channels_[2] = {0, 1};
        // const int histSize_ = 200;
        // const float histRange_[2] = {0, 255};
        // const float *ranges_;
        bool _update_time(cv_bridge::CvImage &cv_image);

    public:
        Tracker();
        cv::Rect2d rect_; // CamShift算法要求要把目标物体的矩形框传递进来
        void get_center(double &u, double &v);
        bool target_init(cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                         const int color); // 目标初始化
        //  目标追踪
        bool target_tracking(cv_bridge::CvImage &cv_image);
        //  更新时间
        // 计算物体速度像素
        bool calculate_speed(double x, double y, double center_x, double center_y,
                             double speed_standard, double &speed);
    };
} // namespace my_hand_eye

#endif // !_TRACKER_H_
