#ifndef _COLOR_TRACKER_H_
#define _COLOR_TRACKER_H_

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <cv_bridge/cv_bridge.h>

#include "my_hand_eye/pose.h"

namespace my_hand_eye
{

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
        Color color_;      // 当前寻找颜色
        const int s_min_[4];
        int white_vmin_;
        bool show_detections_;
        double center_x_, center_y_; // 转盘中心实际位置
        double radius_;              // 半径
        cv::Point2d last_pt_;
        cv::RotatedRect rect_; // 目标物体旋转矩形框
        // 设置旋转矩形
        bool _set_rect(cv_bridge::CvImage &cv_image, cv::Rect &roi);
        // 生成位于图像内的矩形
        cv::Rect _rect_in_image(cv_bridge::CvImage &cv_image, int c_start, int r_start,
                                int c_end, int r_end);
        // 预测矩形框
        cv::Rect _predict_rect(cv_bridge::CvImage &cv_image, Pos &ps, cv::Rect rect_ori,
                               double this_theta, double z, bool read);
        //  更新时间
        bool _update_time(cv_bridge::CvImage &cv_image);

    public:
        bool flag;       // 顺/逆时针标志，flag_为true时逆时针，theta增大，默认顺时针
        int left_color;  // 左侧颜色
        int right_color; // 右侧颜色
        // 颜色范围用于接收yolov5产生的参数
        std::vector<double> h_max;
        std::vector<double> h_min;
        std::vector<double> v_min;
        ColorTracker();
        void get_center(double &u, double &v);
        bool target_init(ros::NodeHandle &nh,
                         cv_bridge::CvImage &cv_image, vision_msgs::BoundingBox2DArray &objArray,
                         const Color color, int white_vmin, double center_x, double center_y,
                         bool show_detections); // 目标初始化
        // 颜色顺序初始化
        bool order_init(vision_msgs::BoundingBox2DArray &objArray, Pos &ps, double z);
        // 目标追踪
        bool target_track(cv_bridge::CvImage &cv_image, Pos &ps, double z);
        // 计算物体半径和速度
        bool calculate_radius_and_speed(double x, double y, double &radius,
                                        double speed_standard_static, double &speed);
        bool no_obstacles(); // 无障碍物
    };
} // namespace my_hand_eye

#endif // !_COLOR_TRACKER_H_
