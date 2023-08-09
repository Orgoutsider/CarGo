#ifndef _ELLIPSE_H_
#define _ELLIPSE_H_

#include "fast_ellipse_detector/EllipseDetectorYaed.h"
#include "my_hand_eye/color_tracker.h"

namespace my_hand_eye
{
    struct Ellipse
    {
        cv::Point2d center; // 目标椭圆容器
        cv::Rect rect_target;
        int color;
        double score_aver; // 分数总和排名前3
        double hypothesis;
    };

    struct EllipseColor
    {
        int color;
        double center_x;
    };

    class EllipseArray : public ColorMethod
    {
    private:
        const int red_hmin_ = 10;
        const int red_hmax_ = 156;
        const int green_hmin_ = 35;
        const int green_hmax_ = 77;
        const int blue_hmin_ = 100;
        const int blue_hmax_ = 124;
        int upper_bound_[4];
        int lower_bound_[4];
        std::vector<Ellipse> ellipse_;
        std::vector<int> flag_; // 聚类标识
        // 目标区域框选
        double color_hypothesis(double h_val, int lower_bound, int upper_bound);

    public:
        EllipseArray();
        // 聚类
        bool clustering(std::vector<cv::Ellipse> &ellipses);
        bool generate_bounding_rect(std::vector<cv::Ellipse> &m_ellipses,
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
