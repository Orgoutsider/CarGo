#include "my_hand_eye/square.h"

namespace my_hand_eye
{
    Square::Square() : length(0) {}

    Square::Square(std::vector<cv::Point> &contour)
    {
        cv::approxPolyDP(contour, approx_, cv::arcLength(contour, true) * 0.03, true); // 多边形拟合
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

    void Square::_square_point_delete()
    {
        int approxSize = approx_.size();
        while (approxSize > 4)
        {
            // len存放长度，point存放端点标号
            std::vector<int> len;
            std::vector<cv::Point2i> point;

            // 计算边长长度
            for (size_t i = 0; i < approxSize; i++)
            {
                len.push_back(sqrt(pow((approx_[i].x - approx_[(i + 1) % approxSize].x), 2) +
                                   pow((approx_[i].y - approx_[(i + 1) % approxSize].y), 2)));
                point.push_back(cv::Point(i, (i + 1) % approxSize));
            }
            // 寻找最短边长的标号
            int len_temp = len[0];
            int len_flag = 0;   // 最短边长的标号
            int point_flag = 0; // 目标点的标号
            for (size_t i = 0; i < approxSize; i++)
            {
                if (len_temp > len[i])
                {
                    len_flag = i;
                    len_temp = len[i];
                }
            }
            // 比较最短边前一条边和后一条边的长度
            int front_side = (len_flag - 1) < 0 ? (approxSize - 1) : (len_flag - 1); // 前一条边的序号
            int back_side = (len_flag + 1) > (approxSize - 1) ? 0 : (len_flag + 1);  // 后一条边的序号
            // 得到目标点的标号
            point_flag =
                len[front_side] < len[back_side]
                    ? point[len_flag].x
                    : point[len_flag].y;
            // 删除目标点
            approx_.erase(approx_.begin() + point_flag);
            approxSize--;
        }
    }

    void Square::_square_point_add()
    {
        int approxSize = approx_.size();
        if (approxSize != 5)
            return;
        // lenth存放长度，point存放端点标号
        std::vector<int> lenth;
        std::vector<cv::Point2i> point;

        // 计算边长长度
        for (size_t i = 0; i < approxSize; i++)
        {
            lenth.push_back(sqrt(pow((approx_[i].x - approx_[(i + 1) % approxSize].x), 2) +
                                 pow((approx_[i].y - approx_[(i + 1) % approxSize].y), 2)));
            point.push_back(cv::Point(i, (i + 1) % approxSize));
        }
        // 寻找最短边长的标号
        int lenth_temp = lenth[0];
        int lenth_flag = 0; // 最短边长的标号
        int lenth_temp2 = lenth[0]; // 次短边
        for (size_t i = 0; i < approxSize; i++)
        {
            if (lenth_temp > lenth[i])
            {
                lenth_flag = i;
                lenth_temp2 = lenth_temp;
                lenth_temp = lenth[i];
            }
            else if (lenth_temp2 > lenth[i])
            {
                lenth_temp2 = lenth[i];
            }
        }
        if (lenth_temp2 * 0.4 < lenth_temp)
            return;
        int front_side = (lenth_flag - 1) < 0 ? (approxSize - 1) : (lenth_flag - 1); // 前一条边的序号
        int back_side = (lenth_flag + 1) > (approxSize - 1) ? 0 : (lenth_flag + 1);  // 后一条边的序号
        cv::Point P1 = approx_[point[front_side].x];
        cv::Point P2 = approx_[point[lenth_flag].x];
        cv::Point P3 = approx_[point[lenth_flag].y];
        cv::Point P4 = approx_[point[back_side].y];
        cv::Point NewPoint;

        double k1 = (double)(P2.y - P1.y) / (P2.x - P1.x);
        double k2 = (double)(P4.y - P3.y) / (P4.x - P3.x);
        NewPoint.x = (double)(k1 * P1.x - k2 * P3.x + P3.y - P1.y) / (k1 - k2);
        NewPoint.y = (double)k1 * (NewPoint.x - P1.x) + P1.y;

        if (point[lenth_flag].x == approxSize - 1)
        {
            approx_.erase(approx_.begin() + point[lenth_flag].x);
            approx_.erase(approx_.begin());
            approx_.insert(approx_.begin(), NewPoint);
        }
        else
        {
            approx_.erase(approx_.begin() + point[lenth_flag].x);
            approx_.erase(approx_.begin() + point[lenth_flag].x);
            approx_.insert(approx_.begin() + point[lenth_flag].x, NewPoint);
        }
    }

    bool Square::_is_quadrilateral()
    {
        if (approx_.size() == 4 && isContourConvex(approx_))
            return true;
        else if (approx_.size() > 4 && !isContourConvex(approx_))
        {
            _square_point_delete();
            return isContourConvex(approx_);
        }
        else if (approx_.size() == 5 && isContourConvex(approx_))
        {
            _square_point_add();
            return approx_.size() == 4;
        }
        else
            return false;
        // return approx_.size() == 4 && isContourConvex(approx_); // 四边形判断
    }

    bool Square::_is_rectangle()
    {
        if (!_is_quadrilateral())
            return false;
        double maxCosine = 0;
        for (int j = 2; j < 5; j++)
        {
            double cosine = fabs(_cosine(j % 4, j - 2, j - 1));
            maxCosine = MAX(maxCosine, cosine);
        }
        // ROS_INFO_STREAM("1 " << maxCosine);
        // 轮廓角度的最大余弦，值越小角度越接近90，判断条件越苛刻
        return maxCosine < 0.85;
    }

    bool Square::is_square()
    {
        if (!_is_rectangle())
            return false;
        return fabs(length * length - area) < length * length / 3.0;
    }

    cv::Point2d Square::_center()
    {
        double k0 = (double)(approx_[2].y - approx_[0].y) / (approx_[2].x - approx_[0].x);
        double k1 = (double)(approx_[3].y - approx_[1].y) / (approx_[3].x - approx_[1].x);
        double x_ = (double)(approx_[1].y - approx_[0].y + k0 * approx_[0].x - k1 * approx_[1].x) / (k0 - k1);
        double y_ = (double)(k0 * (x_ - approx_[0].x) + approx_[0].y);
        return cv::Point2d(x_, y_);
    }

    bool Square::get_pose(geometry_msgs::Pose2D &pose)
    {
        if (approx_.size() != 4)
        {
            ROS_ERROR("Assertion failed: approx_.size() == 4");
            return false;
        }
        cv::Point2d center = _center();
        double sum;
        for (cv::Point &pt : approx_)
        {
            sum += atan2(pt.y - center.y, pt.x - center.x);
        }
        pose.theta = sum / approx_.size();
        pose.x = center.x;
        pose.y = center.y;
        return true;
    }

    BestSquare::BestSquare(std::vector<std::vector<cv::Point>> &contours, double ratio)
    {
        for (std::vector<cv::Point> &contour : contours)
        {
            Square s(contour);
            // ROS_INFO_STREAM(s.area / ratio / ratio << " " << s.length / ratio);
            // 正方形判断和边长排序（起停区边长30cm）
            if (s.area > 100 * ratio * ratio && s.is_square() &&
                fabs(s.length - 30 * ratio) < fabs(best.length - 30 * ratio))
            {
                best = s;
                // best_id = i;
            }
        }
    }

    void SquareMethod::init_hsv(int h_min, int h_max, int s_min, int v_min)
    {
        h_min_ = h_min;
        h_max_ = h_max;
        s_min_ = s_min;
        v_min_ = v_min;
    }

    cv::Mat SquareMethod::square_find(const cv::Mat &img)
    {
        using namespace cv;
        Mat srcgray;
        cvtColor(img, srcgray, COLOR_BGR2GRAY); // 灰度转换
        // imshow("gray", srcgray);
        // waitKey(1);
        Mat srcbinary;
        cv::threshold(srcgray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY); // 阈值化
        // imshow("threshold", srcbinary);
        // waitKey(1);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7), cv::Point(-1, -1));
        morphologyEx(srcbinary, srcbinary, MORPH_CLOSE, kernel, cv::Point(-1, -1), 2); // 闭操作去除噪点
        morphologyEx(srcbinary, srcbinary, MORPH_OPEN, kernel, cv::Point(-1, -1), 2);  // 开操作去除缺口
        // 保证轮廓封闭
        Mat FImg = cv::Mat(srcbinary.size(), CV_8UC1, cv::Scalar::all(0));
        int rows = FImg.rows;
        int cols = FImg.cols;
        // 行处理
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < 1; j++)
            {
                FImg.at<uchar>(i, j) = 255;
                // FImg.at<uchar>(i, cols - 1 - j) = 255;
            }
        }
        // 列处理
        for (size_t i = 0; i < cols; i++)
        {
            for (size_t j = 0; j < 1; j++)
            {
                FImg.at<uchar>(j, i) = 255;
                FImg.at<uchar>(rows - 1 - j, i) = 255;
            }
        }
        bitwise_or(FImg, srcbinary, srcbinary);
        // imshow("MORPH_OPEN", srcbinary);
        // waitKey(1);
        Mat edges;
        Canny(srcbinary, edges, 0, 50, 3, false); // 查找边缘
        // imshow("edges", edges);
        // waitKey(1);
        return edges;
    }

    cv::Mat SquareMethod::square_find_color(const cv::Mat &img)
    {
        // Low of S can be adjusted.High of S and V must be set to 255.
        // cv::Scalar low_Area_Color = cv::Scalar(95, 46, 20);
        // cv::Scalar high_Area_Color = cv::Scalar(130, 255, 255);
        cv::Scalar low_Area_Color = cv::Scalar(h_min_, s_min_, v_min_);
        cv::Scalar high_Area_Color = cv::Scalar(h_max_, 255, 255);
        cv::Mat srcHSV;
        cv::cvtColor(img, srcHSV, cv::COLOR_BGR2HSV);
        cv::Mat StopArea;
        cv::inRange(srcHSV, low_Area_Color, high_Area_Color, StopArea);
        cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
        cv::morphologyEx(StopArea, StopArea, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);  // 开操作去除噪点
        cv::morphologyEx(StopArea, StopArea, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2); // 闭操作去除缺口
        // cv::imshow("StopArea", StopArea);
        // cv::waitKey(1);
        return StopArea;
    }

} // namespace my_hand_eye
