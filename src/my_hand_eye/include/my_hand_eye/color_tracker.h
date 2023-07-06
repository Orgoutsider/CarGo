#ifndef _COLOR_TRACKER_H_
#define _COLOR_TRACKER_H_

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <cv_bridge/cv_bridge.h>

#include "my_hand_eye/pose.h"

namespace my_hand_eye
{
    enum
    {
        color_red = 1,
        color_green,
        color_blue
    };

    class ColorMethod
    {
    private:
        // 将色相值映射为角度
        Angle hue_value(double h_val);
        // 正切的计算
        double hue_value_tan(double y, double x);

    protected:
        const int white_smax_ = 30;

    public:
        // 色相平均值的计算
        double hue_value_aver(cv::Mat &&roi, int white_vmin, cv::Mat &mask_img);
        // 两色相的最小差值
        double hue_value_diff(double h_val1, double h_val2);
    };

    class ColorTracker : public ColorMethod
    {
    private:
        ros::Time last_time_;
        ros::Time this_time_;
        double gain_;      // 矩形框扩大
        double speed_max_; // 最高速度
        int color_;        // 当前寻找颜色
        const double h_max_[4];
        const double h_min_[4];
        int white_vmin_;
        int s_min_;
        int v_min_;
        bool flag_;            // 顺/逆时针标志，flag_为true时逆时针，theta增大，默认顺时针
        cv::RotatedRect rect_; // 目标物体旋转矩形框
        // 设置旋转矩形
        bool _set_rect(cv::Mat &src, cv::Rect &roi);

    protected:
        cv::Point2d last_pt_;
        //  更新时间
        bool _update_time(cv_bridge::CvImage &cv_image);

    public:
        double center_x_, center_y_; // 转盘中心实际位置
        ColorTracker();
        void get_center(double &u, double &v);
        bool target_init(cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                         const int color, int white_vmin, double center_x, double center_y); // 目标初始化
        // 目标追踪
        bool target_track(cv_bridge::CvImage &cv_image, Pos &ps, double z);
        // 计算物体速度
        bool calculate_speed(double x, double y,
                             double speed_standard, double &speed);
    };
} // namespace my_hand_eye

#endif // !_COLOR_TRACKER_H_
