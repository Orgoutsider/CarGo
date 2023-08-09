#include "my_hand_eye/ellipse.h"

namespace my_hand_eye
{
    EllipseArray::EllipseArray()
        : upper_bound_{0, red_hmax_, green_hmax_, blue_hmax_},
          lower_bound_{0, red_hmin_, green_hmin_, blue_hmin_}
    {
        ellipse_.reserve(20);
    };

    bool EllipseArray::clustering(std::vector<cv::Ellipse> &ellipses)
    {
        if (ellipses.empty())
        {
            ROS_WARN("Could not find ellipse.");
            return false;
        }
        if (!ellipse_.empty())
            ellipse_.clear();
        const int MAXN = ellipses.size() + 10;
        flag_.resize(MAXN, 0);

        // 聚类
        for (int i = 0; i < ellipses.size(); i++)
        {
            if (flag_[i])
                continue;
            double x_temp = ellipses[i]._xc, y_temp = ellipses[i]._yc, score_sum = ellipses[i]._score;
            int now = i, cnt = 1;
            for (int j = i + 1; j < ellipses.size(); j++)
            {
                double thr = std::min<float>(ellipses[i]._b, ellipses[j]._b) / 2;
                if ((abs(ellipses[i]._xc - ellipses[j]._xc) < thr) &&
                    (abs(ellipses[i]._yc - ellipses[j]._yc) < thr) &&
                    (abs(ellipses[i]._a / ellipses[i]._b - ellipses[j]._a / ellipses[j]._b) < 0.7) &&
                    !flag_[j])
                {
                    flag_[now] = j;
                    now = j;
                    x_temp += ellipses[j]._xc * (ellipses[j]._score);
                    y_temp += ellipses[j]._yc * (ellipses[j]._score);
                    score_sum += ellipses[j]._score;
                    cnt++;
                }
                if (j == ellipses.size() - 1)
                    flag_[now] = -1;
            }
            if (i == ellipses.size() - 1)
                flag_[now] = -1;
            if (cnt >= 2)
            {
                Ellipse e;
                // 平均数求聚类中心，感觉不太妥当，但是精度感觉还行，追求精度的话可以用 Weiszfeld 算法求中位中心，那个要迭代
                e.center = cv::Point2d(x_temp / score_sum, y_temp / score_sum);
                e.score_aver = score_sum / cnt;
                ellipse_.push_back(e);
            }
        }
        // ROS_INFO_STREAM(ellipse_.size());
        return true;
    };

    bool EllipseArray::generate_bounding_rect(std::vector<cv::Ellipse> &m_ellipses,
                                              cv_bridge::CvImagePtr &cv_image)
    {
        if (m_ellipses.empty() || ellipse_.empty())
            return false;
        bool note[flag_.size()] = {false};
        // for (int i = 0; i < m_ellipses.size(); i++)
        //     ROS_INFO_STREAM(flag_[i]);
        int num = 0;
        float ratio = 0.1f;
        for (int i = 0; i < m_ellipses.size(); i++)
        {
            if (note[i])
                continue;
            int now = i, cnt = 1, ind_max = i;
            float area_max = m_ellipses[now]._a * m_ellipses[now]._b * m_ellipses[now]._score;
            // ROS_INFO_STREAM(now);
            note[now] = true;
            while (flag_[now] != -1)
            {
                cnt++;
                now = flag_[now];
                note[now] = true;
                float area_temp = m_ellipses[now]._a * m_ellipses[now]._b * m_ellipses[now]._score;
                area_max = std::max(area_max, area_temp);
                ind_max = (area_max > area_temp) ? ind_max : now;
            }
            if (cnt >= 2)
            {
                cv::Rect rect = cv::Rect(cvFloor(m_ellipses[ind_max]._xc - m_ellipses[ind_max]._a *
                                                                               (1 + ratio)),
                                         cvFloor(m_ellipses[ind_max]._yc - m_ellipses[ind_max]._a *
                                                                               (1 + ratio)),
                                         cvCeil(m_ellipses[ind_max]._a) * 2 * (1 + ratio),
                                         cvCeil(m_ellipses[ind_max]._a) * 2 * (1 + ratio));
                rect.x = std::max(rect.x, 0);
                rect.y = std::max(rect.y, 0);
                rect.width = std::min(rect.width, cv_image->image.cols - rect.x);
                rect.height = std::min(rect.height, cv_image->image.rows - rect.y);
                try
                {
                    if (!ellipse_.at(num).center.inside(rect))
                        rect.width = rect.height = 0;
                    ellipse_.at(num).rect_target = rect;
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("generate bounding rect error: %s", e.what());
                    return false;
                }
                num++;
            }
        }
        if (num != ellipse_.size())
        {
            ROS_ERROR("Incorrect num! num:%d, ellipse size:%ld", num, ellipse_.size());
            return false;
        }
        return true;
    }

    bool EllipseArray::color_classification(cv_bridge::CvImagePtr &cv_image,
                                            int white_vmin)
    {
        if (ellipse_.empty() || cv_image->image.empty())
            return false;
        for (Ellipse &e : ellipse_)
        {
            cv::Mat mask;
            // 保存当前区域色相H的平均值
            double H_Average = -1;
            if (!e.rect_target.empty())
                H_Average = hue_value_aver(cv_image->image(e.rect_target), white_vmin, mask);
            // ROS_INFO_STREAM("H_Average:" << H_Average);
            int id = color_red;
            double max_hyp = 0;
            for (int color = color_red; color <= color_blue; color++)
            {
                double hyp = color_hypothesis(H_Average, lower_bound_[color], upper_bound_[color]);
                if (hyp > max_hyp)
                {
                    id = color;
                    max_hyp = hyp;
                }
            }
            e.color = id;
            e.hypothesis = max_hyp;
        }
        return true;
    }

    bool EllipseArray::detection(vision_msgs::BoundingBox2DArray &objArray,
                                 cv::Rect &roi, cv_bridge::CvImagePtr &cv_image, bool show_detection)
    {
        if (ellipse_.empty())
            return false;
        if (!objArray.boxes.empty())
            objArray.boxes.clear();
        objArray.boxes.resize(4);
        double hyp_max[4] = {0};
        for (Ellipse &e : ellipse_)
        {
            // ROS_INFO_STREAM(e.hypothesis << hyp_max[e.color]);
            if ((e.hypothesis + e.score_aver) * 0.5 > hyp_max[e.color])
            {
                vision_msgs::BoundingBox2D obj = vision_msgs::BoundingBox2D();
                if (!show_detection)
                {
                    obj.center.x = e.center.x / cv_image->image.cols * roi.width + roi.x;
                    obj.center.y = e.center.y / cv_image->image.rows * roi.height + roi.y;
                }
                else
                {
                    obj.center.x = e.center.x;
                    obj.center.y = e.center.y;
                    obj.size_x = e.rect_target.width;
                    obj.size_y = e.rect_target.height;
                }
                objArray.boxes[e.color] = obj;
                hyp_max[e.color] = e.hypothesis;
            }
        }
        if (show_detection && !cv_image->image.empty())
        {
            for (int color = color_red; color <= color_blue; color++)
            {
                if (!objArray.boxes[color].center.x)
                    continue;
                // 绘制中心十字，用于调试
                cv::Scalar c;
                switch (color)
                {
                case color_red:
                    c = cv::Scalar(0, 0, 255);
                    break;

                case color_green:
                    c = cv::Scalar(0, 255, 0);
                    break;

                case color_blue:
                    c = cv::Scalar(255, 0, 0);
                    break;

                default:
                    break;
                }
                cv::Point pt1(objArray.boxes[color].center.x - objArray.boxes[color].size_x / 2,
                              objArray.boxes[color].center.y - objArray.boxes[color].size_y / 2);
                cv::Point pt2(objArray.boxes[color].center.x + objArray.boxes[color].size_x / 2,
                              objArray.boxes[color].center.y + objArray.boxes[color].size_y / 2);
                cv::rectangle(cv_image->image, pt1, pt2, cv::Scalar(0, 0, 0), 1, 4);
                draw_cross(cv_image->image,
                           cv::Point2d(objArray.boxes[color].center.x, objArray.boxes[color].center.y),
                           c, 15, 1);
                objArray.boxes[color].center.x = objArray.boxes[color].center.x /
                                                     cv_image->image.cols * roi.width +
                                                 roi.x;
                objArray.boxes[color].center.y = objArray.boxes[color].center.y /
                                                     cv_image->image.rows * roi.height +
                                                 roi.y;
                // imshow("srcCopy", cv_image->image); // 用于调试
                // cv::waitKey(60);
            }
        }
        return true;
    };

    double EllipseArray::color_hypothesis(double h_val, int lower_bound, int upper_bound)
    {
        if (h_val > 180 || h_val < 0)
            return 0;
        double lower_diff = hue_value_diff(h_val, lower_bound);
        double upper_diff = hue_value_diff(h_val, upper_bound);
        double diff = hue_value_diff(upper_bound, lower_bound);
        if (abs(lower_diff + upper_diff - diff) < 1)
        {
            return 1;
        }
        else if (abs(lower_diff + upper_diff + diff - 180) < 1 ||
                 abs(std::min<double>(lower_diff, upper_diff) + diff -
                     std::max<double>(lower_diff, upper_diff)) < 1)
        {
            return exp(-std::min<double>(lower_diff, upper_diff) / diff);
        }
        else
        {
            ROS_ERROR("Diff error! diff:%lf lower_diff:%lf upper_diff:%lf", diff, lower_diff, upper_diff);
            ROS_ERROR("h_val:%lf lower_bound:%d upper_bound:%d", h_val, lower_bound, upper_bound);
            return 0;
        }
    }

    void draw_cross(cv::Mat &img, cv::Point2d point, cv::Scalar color, int size, int thickness)
    {
        if (!point.inside(cv::Rect2d(0, 0, img.cols, img.rows)))
            ROS_WARN("Point is out of range! point:(%lf, %lf)", point.x, point.y);
        // 绘制横线
        line(img, cv::Point(point.x - size / 2, point.y),
             cv::Point(point.x + size / 2, point.y), color, thickness, 8, 0);
        // 绘制竖线
        line(img, cv::Point(point.x, point.y - size / 2),
             cv::Point(point.x, point.y + size / 2), color, thickness, 8, 0);
    }
} // namespace my_hand_eye
