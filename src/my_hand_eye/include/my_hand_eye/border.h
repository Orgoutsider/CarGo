#ifndef _BORDER_H_
#define _BORDER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

namespace my_hand_eye
{
    class BorderMethod
    {
    public:
        void plot_line(cv::Mat &mat, double rho, double theta, cv::Scalar color, double width = 1);
        double color_judge(cv::Vec2f &line, cv::Mat &thre_img);
        // 颜色增强函数
        cv::Mat saturation(cv::Mat &src, int percent);
        // 颜色inrange查找车道边界
        cv::Mat LBD_color_func(cv::Mat &enhanced, int threshold);
        // gramma矫正
        void gramma_transform(int factor, cv::Mat &img);
    };

    class Border : public BorderMethod
    {
    private:
        float theta_thr_horizontal_;
        // 聚类
        bool cluster_lines(cv::Vec2f lines_sel[], const int cnt,
                           cv_bridge::CvImagePtr &cv_image, cv::Vec2f &border,
                           bool show_detection);

    public:
        Border();
        enum Detected
        {
            detected_yellow,
            detected_grey,
            detected_both
        };
        // 主函数
        bool detect(cv_bridge::CvImagePtr &cv_image, cv::Vec2f &border, cv::Rect &rect, Detected &detected,
                    int threshold, bool show_detection, sensor_msgs::ImagePtr &debug_image);
    };
} // namespace my_hand_eye

#endif // !_BORDER_H_
