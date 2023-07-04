#ifndef _BORDER_H_
#define _BORDER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

namespace my_hand_eye
{
    class Border
    {
    private:
        float theta_thr_horizontal_;
        void plot_line(cv::Mat &mat, double rho, double theta, cv::Scalar color, double width = 1);
        cv::Mat saturation(cv::Mat &src, int percent); // 颜色增强函数
    public:
        Border();
        // 主函数
        bool find(cv_bridge::CvImagePtr &cv_image, cv::Vec2f &border,
                  boost::function<void(cv::Mat &, std::vector<cv::Vec2f> &)> LBD, bool show_detection);
    };

    // 自适应阈值化查找车道边界
    void LBD_thershold_func(cv::Mat &enhanced, std::vector<cv::Vec2f> &lines);
    // 颜色inrange查找车道边界
    void LBD_color_func(cv::Mat &enhanced, std::vector<cv::Vec2f> &lines);
} // namespace my_hand_eye

#endif // !_BORDER_H_
