#ifndef _ELLIPSE_H_
#define _ELLIPSE_H_

#include "my_hand_eye/tracker.h"
#include "my_hand_eye/pose.h"

namespace my_hand_eye
{
    struct Ellipse
    {
        cv::Point2d center; // 目标椭圆容器
        cv::Rect rect_target;
        int color;
        double hypothesis;
    };

    struct EllipseColor
    {
        int color;
        double center_x;
    };

    class EllipseArray
    {
    private:
        std::vector<Ellipse> ellipse_;
        std::vector<int> flag_; // 聚类标识
        // 将色相值映射为角度
        Angle hue_value(double h_val);
        // 色相平均值的计算
        double hue_value_tan(double y, double x);
        // 两色相的最小差值
        double hue_value_diff(double h_val1, double h_val2);
        // 目标区域框选
        double color_hypothesis(double h_val, int lower_bound, int upper_bound);

    public:
        EllipseArray();
        // 聚类
        bool clustering(std::vector<cv::Point2d> &centers, std::vector<cv::RotatedRect> &ellipses);
        bool generate_bounding_rect(std::vector<cv::RotatedRect> &m_ellipses,
                                    cv_bridge::CvImagePtr &cv_image);
        // 颜色分类
        bool color_classification(cv_bridge::CvImagePtr &cv_image,
                                  int white_vmin);
        // 找到最多3个椭圆并绘制
        bool detection(vision_msgs::BoundingBox2DArray &objArray,
                       cv::Rect &roi, cv_bridge::CvImagePtr &cv_image, bool show_detection);
    };

    // 十字光标绘制，用于调试观察
    void draw_cross(cv::Mat &img, cv::Point2d point, cv::Scalar color, int size, int thickness);
} // namespace my_hand_eye

#endif // !_ELLIPSE_H_
