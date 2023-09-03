#include "my_hand_eye/border.h"
#include "my_hand_eye/backward_kinematics.h"

namespace my_hand_eye
{
    Border::Border() : theta_thr_horizontal_(16) {}

    void BorderMethod::plot_line(cv::Mat &mat, double rho, double theta, cv::Scalar color, double width)
    {
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        cv::line(mat, pt1, pt2, color, width, cv::LINE_AA);
    }

    double BorderMethod::color_judge(cv::Vec2f &line, cv::Mat &thre_img)
    {
        if (thre_img.empty())
            return 0;
        double rho = line[0], theta = line[1];
        double rho_grey = rho + 3, rho_yellow = rho - 3;
        cv::Mat line_grey = cv::Mat::zeros(thre_img.size(), CV_8UC1);
        cv::Mat line_yellow = line_grey.clone();
        plot_line(line_grey, rho_grey, theta, cv::Scalar(255), 3);
        plot_line(line_yellow, rho_yellow, theta, cv::Scalar(255), 3);
        // cv::imshow("line_grey", line_grey);
        // cv::imshow("line_yellow", line_yellow);
        // cv::waitKey(10);
        cv::Mat mask_grey, mask_yellow;
        cv::bitwise_and(line_grey, thre_img, mask_grey);
        cv::subtract(line_yellow, thre_img, mask_yellow, cv::noArray(), CV_8UC1);
        // cv::imshow("mask_grey", mask_grey);
        // cv::imshow("mask_yellow", mask_yellow);
        return 0.5 * ((cv::countNonZero(mask_grey) + 1.0) / (cv::countNonZero(line_grey) + 1.0) +
                      (cv::countNonZero(mask_yellow) + 1.0) / (cv::countNonZero(line_yellow) + 1.0));
    }

    cv::Mat BorderMethod::saturation(cv::Mat &src, int percent)
    {
        float Increment = percent * 1.0f / 100;
        cv::Mat temp = src.clone();
        int row = src.rows;
        int col = src.cols;
        for (int i = 0; i < row; ++i)
        {
            uchar *t = temp.ptr<uchar>(i);
            uchar *s = src.ptr<uchar>(i);
            for (int j = 0; j < col; ++j)
            {
                uchar b = s[3 * j];
                uchar g = s[3 * j + 1];
                uchar r = s[3 * j + 2];
                float max = std::max({r, g, b});
                float min = std::min({r, g, b});
                float delta, value;
                float L, S, alpha;
                delta = (max - min) / 255;
                if (delta == 0)
                    continue;
                value = (max + min) / 255;
                L = value / 2;
                if (L < 0.5)
                    S = delta / value;
                else
                    S = delta / (2 - value);
                if (Increment >= 0)
                {
                    if ((Increment + S) >= 1)
                        alpha = S;
                    else
                        alpha = 1 - Increment;
                    alpha = 1 / alpha - 1;
                    t[3 * j + 2] = static_cast<uchar>(r + (r - L * 255) * alpha);
                    t[3 * j + 1] = static_cast<uchar>(g + (g - L * 255) * alpha);
                    t[3 * j] = static_cast<uchar>(b + (b - L * 255) * alpha);
                }
                else
                {
                    alpha = Increment;
                    t[3 * j + 2] = static_cast<uchar>(L * 255 + (r - L * 255) * (1 + alpha));
                    t[3 * j + 1] = static_cast<uchar>(L * 255 + (g - L * 255) * (1 + alpha));
                    t[3 * j] = static_cast<uchar>(L * 255 + (b - L * 255) * (1 + alpha));
                }
            }
        }
        return temp;
    }

    bool Border::cluster_lines(cv::Vec2f lines_sel[], const int cnt,
                               cv_bridge::CvImagePtr &cv_image, cv::Vec2f &border,
                               bool show_detection)
    {
        border[0] = border[1] = 0;
        if (!cnt)
            return false;
        float sum = lines_sel[0][0];
        int size = 1;
        bool flag = true; // 有新线加入
        int note[cnt] = {0};
        note[0] = 1;
        const float var_thr = 0.002 * cv_image->image.rows * cv_image->image.rows;
        while (flag && ros::ok())
        {
            flag = false;
            for (int i = 1; i < cnt; i++)
            {
                if (note[i])
                    continue;
                sum += lines_sel[i][0];
                size++;
                float aver = sum / size;
                // 计算方差
                float dist_max = (lines_sel[i][0] - aver) * (lines_sel[i][0] - aver),
                      var = dist_max;
                int ind_max = i;
                for (int j = 0; j < cnt; j++)
                {
                    if (note[j] != 1)
                        continue;
                    float dist = (lines_sel[j][0] - aver) * (lines_sel[j][0] - aver);
                    if (dist > dist_max) // 找距离中心最远点
                    {
                        dist_max = dist;
                        ind_max = j;
                    }
                    var += dist;
                }
                var /= size;
                if (var < var_thr)
                {
                    note[i] = 1;
                    flag = true;
                    break;
                }
                else if (ind_max != i)
                {
                    // 去掉距离中心最远点再次尝试
                    note[ind_max] = 2;
                    size--;
                    sum -= lines_sel[ind_max][0];
                    float aver = sum / size;
                    // 计算方差
                    float var = (lines_sel[i][0] - aver) * (lines_sel[i][0] - aver);
                    for (int j = 0; j < cnt; j++)
                    {
                        if (note[j] != 1)
                            continue;
                        float dist = (lines_sel[j][0] - aver) * (lines_sel[j][0] - aver);
                        var += dist;
                    }
                    if (var < var_thr)
                    {
                        flag = true;
                        note[i] = 1;
                        break;
                    }
                    note[ind_max] = 1;
                    sum += lines_sel[ind_max][0];
                    sum -= lines_sel[i][0];
                }
                else
                {
                    size--;
                    sum -= lines_sel[i][0];
                }
            }
        }
        for (int i = 0; i < cnt; i++)
        {
            if (note[i] != 1)
            {
                if (show_detection && !cv_image->image.empty())
                {
                    // 排除的直线绘制黑色
                    plot_line(cv_image->image, lines_sel[i][0], lines_sel[i][1], cv::Scalar(0, 0, 0), 1);
                }
                continue;
            }
            border += lines_sel[i];
            if (show_detection && !cv_image->image.empty())
            {
                // 绘制红色
                plot_line(cv_image->image, lines_sel[i][0], lines_sel[i][1], cv::Scalar(0, 0, 255), 1);
            }
        }
        if (size)
            border /= size;
        return size >= 3;
    }

    bool Border::detect(cv_bridge::CvImagePtr &cv_image, cv::Vec2f &border,
                        cv::Rect &rect, Detected &detected, int threshold,
                        bool show_detection, sensor_msgs::ImagePtr &debug_image)
    {
        int f = 2;
        cv::resize(cv_image->image, cv_image->image, cv_image->image.size() / f);
        // 使用原图区分两种颜色
        static bool flag = true;
        static cv::Mat ori;
        cv::Mat ori_now;
        ori_now = cv_image->image.clone();
        cv_image->image = saturation(cv_image->image, 100);                       // 饱和度调整参数，-100 — 100, 正数饱和度增强，负数饱和度减弱
        cv::GaussianBlur(cv_image->image, cv_image->image, cv::Size(5, 5), 0, 0); // 滤波预处理
        std::vector<cv::Vec2f> lines;
        cv::Mat thre_img = LBD_color_func(cv_image->image, threshold);
        cv::Mat edgeImg;
        cv::Canny(thre_img, edgeImg, 50, 200, 3);
        cv::HoughLines(edgeImg, lines, 1, CV_PI / 180, threshold, 0, 0);
        const int MAXN = lines.size() + 5;
        cv::Vec2f lines_sel[MAXN];
        // cv::cvtColor(ori, ori, cv::COLOR_BGR2HSV);
        // cv::Scalar Boundary_low = cv::Scalar(78, 0, 0);
        // cv::Scalar Boundary_high = cv::Scalar(179, 255, 255);
        // cv::inRange(ori, Boundary_low, Boundary_high, ori);
        // cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        // cv::morphologyEx(ori, ori, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 3);
        // cv::imshow("thr", ori);
        // cv::waitKey(10);
        if (lines.empty())
        {
            double grey = -1;
            if (!ori.empty() && !ori_now.empty())
            {
                cv::hconcat(ori_now, ori, ori_now);
                // cv::imshow("ori", ori_now);
                ori_now = saturation(ori_now, 100);
                ori_now = LBD_color_func(ori_now, threshold);
                cv::imshow("ori", ori_now);
                cv::waitKey(10);
                grey = cv::countNonZero(
                           ori_now(cv::Range(0, ori.rows), cv::Range(0, ori.cols))) /
                       (ori.rows * ori.cols * 1.0);
                if (grey > 0.4)
                {
                    detected = detected_grey;
                    return true;
                }
                else if (grey < 0.3)
                {
                    detected = detected_yellow;
                    return true;
                }
            }
            ROS_WARN("Could not find border! grey: %f", grey);
            return false;
        }
        int cnt = 0;
        for (cv::Vec2f &line : lines)
        {
            double rho = line[0], theta = line[1];
            double theta_d = Angle::degree(theta);
            if (abs(theta_d - 90) < theta_thr_horizontal_ && color_judge(line, thre_img) > 0.5)
            {
                lines_sel[cnt++] = line;
            }
            else if (show_detection && !cv_image->image.empty())
            {
                // 排除的直线绘制黑色
                plot_line(cv_image->image, rho, theta, cv::Scalar(0, 0, 0), 1);
            }
        }
        if (!cluster_lines(lines_sel, cnt, cv_image, border, show_detection))
        {
            if (show_detection && !cv_image->image.empty())
            {
                debug_image = cv_image->toImageMsg();
            }
            double grey = -1;
            if (!ori.empty() && !ori_now.empty())
            {
                cv::hconcat(ori_now, ori, ori_now);
                // cv::imshow("ori", ori_now);
                ori_now = saturation(ori_now, 100);
                ori_now = LBD_color_func(ori_now, threshold);
                cv::imshow("thr", ori_now);
                cv::waitKey(10);
                grey = cv::countNonZero(
                           ori_now(cv::Range(0, ori.rows), cv::Range(0, ori.cols))) /
                       (ori.rows * ori.cols * 1.0);
                if (grey > 0.4)
                {
                    detected = detected_grey;
                    return true;
                }
                else if (grey < 0.3)
                {
                    detected = detected_yellow;
                    return true;
                }
            }
            ROS_WARN("Could not find border! grey: %f", grey);
            return false;
        }
        ori = ori_now;
        if (show_detection && !cv_image->image.empty())
        {
            // cv::imshow("Hough", cv_image->image);
            // 白色
            plot_line(cv_image->image, border[0], border[1], cv::Scalar(255, 255, 255), 2);
            debug_image = cv_image->toImageMsg();
        }
        // 对之前resize的恢复
        border[0] *= f;
        border[0] = border[0] + rect.x * cos(border[1]) + rect.y * sin(border[1]);
        detected = detected_both;
        return true;
    }

    cv::Mat BorderMethod::LBD_color_func(cv::Mat &enhanced, int threshold)
    {
        using namespace cv;
        Mat HSVImg;
        cvtColor(enhanced, HSVImg, COLOR_BGR2HSV_FULL);
        // imshow("HSVImg", HSVImg);

        // Scalar Boundary_low = Scalar(78, 0, 0);
        // Scalar Boundary_high = Scalar(179, 255, 255);
        std::vector<Mat> HSV;
        split(HSVImg, HSV);
        Mat RangeImg;
        cv::threshold(HSV[0], RangeImg, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        // imshow("RangeImg", RangeImg);
        // inRange(HSVImg, Boundary_low, Boundary_high, RangeImg);
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(RangeImg, RangeImg, MORPH_CLOSE, element, Point(-1, -1), 3);
        return RangeImg;
    }
} // namespace my_hand_eye
