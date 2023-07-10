#include "my_hand_eye/square.h"

namespace my_hand_eye
{
    Square::Square() : length(0) {}

    Square::Square(std::vector<cv::Point> &contour)
    {
        cv::approxPolyDP(contour, approx_, cv::arcLength(contour, true) * 0.02, true); // 多边形拟合
        length = cv::arcLength(approx_, true) / 4;
        area = fabs(cv::contourArea(approx_));
    }

    double Square::_cosine(int pt0, int pt1, int pt2)
    {
        double dx1 = approx_[pt1].x - approx_[pt0].x;
        double dy1 = approx_[pt1].y - approx_[pt0].y;
        double dx2 = approx_[pt2].x - approx_[pt0].x;
        double dy2 = approx_[pt2].y - approx_[pt0].y;
        return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
    }

    bool Square::_is_quadrilateral()
    {
        return approx_.size() == 4 && isContourConvex(approx_); // 四边形判断
    }

    bool Square::_is_rectangle()
    {
        if (!_is_quadrilateral())
            return false;
        double maxCosine;
        for (int j = 2; j < 5; j++)
        {
            double cosine = fabs(_cosine(j % 4, j - 2, j - 1));
            maxCosine = MAX(maxCosine, cosine);
        }
        // 轮廓角度的最大余弦，值越小角度越接近90，判断条件越苛刻
        return maxCosine < 0.83;
    }

    bool Square::is_square()
    {
        if (!_is_rectangle())
            return false;
        return fabs(length * length - area) < length * length / 3.0;
    }

    cv::Point2d Square::center()
    {
        double k0 = (double)(approx_[2].y - approx_[0].y) / (approx_[2].x - approx_[0].x);
        double k1 = (double)(approx_[3].y - approx_[1].y) / (approx_[3].x - approx_[1].x);
        double x_ = (double)(approx_[1].y - approx_[0].y + k0 * approx_[0].x - k1 * approx_[1].x) / (k0 - k1);
        double y_ = (double)(k0 * (x_ - approx_[0].x) + approx_[0].y);
        return cv::Point2d(x_, y_);
    }

    BestSquare::BestSquare(std::vector<std::vector<cv::Point>> &contours, double ratio)
    {
        for (size_t i = 0; i < contours.size(); i++)
        {
            Square s(contours[i]);
            // 正方形判断和边长排序（起停区边长30cm）
            if (s.area > 100 * ratio * ratio && s.is_square() &&
                fabs(s.length - 30 * ratio) < fabs(best.length - 30 * ratio))
            {
                best = s;
                best_id = i;
            }
        }
    }

} // namespace my_hand_eye
