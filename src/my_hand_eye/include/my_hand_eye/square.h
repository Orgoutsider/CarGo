#ifndef _SQUARE_H_
#define _SQUARE_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace my_hand_eye
{
    class Square
    {
    private:
        std::vector<cv::Point> approx_; // 轮廓角度最大余弦判断
        double _cosine(int pt0, int pt1, int pt2);
        bool _is_quadrilateral(); // 四边形判断
        bool _is_rectangle();     // 矩形判断
    public:
        double length; // 边长
        Square();
        Square(std::vector<cv::Point> &contour);
        bool is_square();   // 正方形判断
        cv::Point2d center(); // 图形中心点计算
    };

    class BestSquare
    {
    public:
        int best_id;
        Square best;
        // 构造函数中查找最优正方形，如果没有正方形，best.length = 0
        BestSquare(std::vector<std::vector<cv::Point>> &contours, double ratio);
    };

} // namespace my_hand_eye

#endif // !_SQUARE_H_
