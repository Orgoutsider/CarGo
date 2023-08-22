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
        // 自适应阈值化查找车道边界
        cv::Mat LBD_thershold_func(cv::Mat &enhanced, std::vector<cv::Vec2f> &lines, int threshold);
        // 颜色inrange查找车道边界
        cv::Mat LBD_color_func(cv::Mat &enhanced, std::vector<cv::Vec2f> &lines, int threshold);
    };

    class Border : public BorderMethod
    {
    private:
        float theta_thr_horizontal_;

    public:
        Border();
        // 主函数
        bool detect(cv_bridge::CvImagePtr &cv_image, cv::Vec2f &border, cv::Rect &rect,
                    boost::function<cv::Mat(cv::Mat &, std::vector<cv::Vec2f> &)> LBD,
                    bool show_detection, sensor_msgs::ImagePtr &debug_image);
    };
} // namespace my_hand_eye

#endif // !_BORDER_H_
