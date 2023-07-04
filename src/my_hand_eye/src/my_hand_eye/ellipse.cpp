#include "my_hand_eye/ellipse.h"

namespace my_hand_eye
{
    EllipseArray::EllipseArray()
    {
        ellipse_.reserve(3);
    };

    bool EllipseArray::clustering(std::vector<cv::Point2d> &centers, std::vector<cv::RotatedRect> &ellipses)
    {
        if (centers.empty() || ellipses.empty())
            return false;
        if (!ellipse_.empty())
            ellipse_.clear();
        const int MAXN = centers.size() + 10;
        flag_.resize(MAXN, 0);
        // ROS_INFO_STREAM(centers.size());
        // 聚类
        for (int i = 0; i < centers.size(); i++)
        {
            if (flag_[i])
                continue;
            int x_temp = centers[i].x, y_temp = centers[i].y, count = 1, now = i;
            for (int j = i + 1; j < centers.size(); j++)
            {
                float thr = std::min<float>(std::min<float>(ellipses[i].size.width, ellipses[i].size.height),
                                            std::min<float>(ellipses[j].size.width, ellipses[j].size.height)) /
                            2;
                if ((abs(centers[i].x - centers[j].x) < thr) &&
                    (abs(centers[i].y - centers[j].y) < thr) &&
                    !flag_[j])
                {

                    flag_[now] = j;
                    now = j;
                    x_temp = x_temp + centers[j].x;
                    y_temp = y_temp + centers[j].y;
                    count++;
                }
                if (j == centers.size() - 1)
                    flag_[now] = -1;
            }
            if (i == centers.size() - 1)
                flag_[now] = -1;
            if (count > 2)
            {
                Ellipse e;
                // 平均数求聚类中心，感觉不太妥当，但是精度感觉还行，追求精度的话可以用 Weiszfeld 算法求中位中心，那个要迭代
                e.center = cv::Point(x_temp / count, y_temp / count);
                ellipse_.push_back(e);
            }
        }
        // ROS_INFO_STREAM(ellipse_.size());
        return true;
    };

    bool EllipseArray::generate_bounding_rect(std::vector<cv::RotatedRect> &m_ellipses,
                                              cv_bridge::CvImagePtr &cv_image)
    {
        if (m_ellipses.empty() || ellipse_.empty())
            return false;
        bool note[flag_.size()] = {false};
        // for (int i = 0; i < m_ellipses.size(); i++)
        //     ROS_INFO_STREAM(flag_[i]);
        int num = 0;
        for (int i = 0; i < m_ellipses.size(); i++)
        {
            if (note[i])
                continue;
            int now = i, area_max = (m_ellipses[now].boundingRect()).area(), cnt = 1, ind_max = i;
            // ROS_INFO_STREAM(now);
            note[now] = true;
            while (flag_[now] != -1)
            {
                cnt++;
                now = flag_[now];
                note[now] = true;
                int area_temp = (m_ellipses[now].boundingRect()).area();
                area_max = (area_max > area_temp) ? area_max : area_temp;
                ind_max = (area_max > area_temp) ? ind_max : now;
            }
            if (cnt > 2)
            {
                cv::Rect rect = m_ellipses[ind_max].boundingRect();
                rect.x = std::max<int>(rect.x, 0);
                rect.y = std::max<int>(rect.y, 0);
                rect.width = std::min<int>(rect.width, cv_image->image.cols - rect.x);
                rect.height = std::min<int>(rect.height, cv_image->image.rows - rect.y);
                try
                {
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
        const int WHITE_SMAX = 46;
        const int WHITE_VMIN = white_vmin;
        const int RED_HMIN = 10;
        const int RED_HMAX = 156;
        const int GREEN_HMIN = 35;
        const int GREEN_HMAX = 77;
        const int BLUE_HMIN = 100;
        const int BLUE_HMAX = 124;
        int upper_bound[] = {0, RED_HMAX, GREEN_HMAX, BLUE_HMAX};
        int lower_bound[] = {0, RED_HMIN, GREEN_HMIN, BLUE_HMIN};
        for (Ellipse &e : ellipse_)
        {
            if (e.rect_target.empty())
            {
                ROS_ERROR("rect target is empty!");
                return false;
            }
            cv::Mat mask = (cv_image->image(e.rect_target)).clone();
            cv::Mat mask_Img = mask.clone();        // 用于调试
            mask_Img = {cv::Scalar(255, 255, 255)}; // 用于调试
            cvtColor(mask, mask, cv::COLOR_BGR2HSV);
            // 设置像素遍历迭代器
            cv::MatConstIterator_<cv::Vec3b> maskStart = mask.begin<cv::Vec3b>();
            cv::MatConstIterator_<cv::Vec3b> maskEnd = mask.end<cv::Vec3b>();
            cv::MatIterator_<cv::Vec3b> mask_ImgStart = mask_Img.begin<cv::Vec3b>(); // 用于调试
            double x = 0, y = 0;
            int cnt = 0;
            for (; maskStart != maskEnd; maskStart++, mask_ImgStart++)
            {
                // 过滤白色
                if ((*maskStart)[1] <= WHITE_SMAX && (*maskStart)[2] >= WHITE_VMIN)
                {
                    // 用于调试
                    (*mask_ImgStart)[0] = 0;
                    (*mask_ImgStart)[1] = 0;
                    (*mask_ImgStart)[2] = 0;
                    continue;
                }
                int H_Val = (*maskStart)[0];
                Angle color = hue_value(H_Val);
                x += color.cos();
                y += color.sin();
                cnt++;
                // if (cnt % 200 == 0)
                // {
                //     ROS_INFO_STREAM(H_Val);
                //     usleep(1e5);
                // }
            }
            // cv::imshow("mask_Img", mask_Img); // 用于调试
            // cv::waitKey(10);
            double H_Average = hue_value_tan(y, x); // 保存当前区域色相H的平均值
            H_Average = (H_Average < 0) ? H_Average + 180 : H_Average;
            // ROS_INFO_STREAM("H_Average:" << H_Average << " cnt:" << cnt << " " << mask.cols * mask.rows);
            int id = color_red;
            double max_hyp = 0;
            for (int color = color_red; color <= color_blue; color++)
            {
                if (color_hypothesis(H_Average, lower_bound[color], upper_bound[color]) > max_hyp)
                {
                    id = color;
                    max_hyp = color_hypothesis(H_Average, lower_bound[color], upper_bound[color]);
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
            if (e.hypothesis > hyp_max[e.color])
            {
                vision_msgs::BoundingBox2D obj = vision_msgs::BoundingBox2D();
                obj.center.x = e.center.x + roi.x;
                obj.center.y = e.center.y + roi.y;
                obj.size_x = e.rect_target.width;
                obj.size_y = e.rect_target.height;
                objArray.boxes[e.color] = obj;
                hyp_max[e.color] = e.hypothesis;
                if (show_detection)
                {
                    // 绘制中心十字，用于调试
                    cv::Scalar c;
                    switch (e.color)
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
                    draw_cross(cv_image->image,
                               e.center,
                               c, 30, 2);
                    // imshow("srcCopy", cv_image->image); // 用于调试
                    // cv::waitKey(60);
                }
            }
        }
        return true;
    };

    Angle EllipseArray::hue_value(double h_val)
    {
        return Angle(h_val * 2);
    }

    double EllipseArray::hue_value_tan(double y, double x)
    {
        return Angle(y, x)._get_degree() / 2;
    }

    double EllipseArray::hue_value_diff(double h_val1, double h_val2)
    {
        Angle a1 = hue_value(h_val1);
        Angle a2 = hue_value(h_val2);
        double res = abs((a1 - a2)._get_degree());
        if (res > 180)
        {
            res = 360.0 - res;
        }
        return res / 2;
    }

    double EllipseArray::color_hypothesis(double h_val, int lower_bound, int upper_bound)
    {
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
