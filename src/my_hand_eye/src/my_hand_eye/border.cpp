#include "my_hand_eye/border.h"
#include "my_hand_eye/backward_kinematics.h"

namespace my_hand_eye
{
    Border::Border() : theta_thr_horizontal_(16) {}

    void Border::plot_line(cv::Mat &mat, double rho, double theta, cv::Scalar color, double width)
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

    bool Border::find(cv_bridge::CvImagePtr &cv_image, cv::Vec2f &border,
                      boost::function<void(cv::Mat &, std::vector<cv::Vec2f> &)> LBD,
                      bool show_detection, sensor_msgs::ImagePtr &debug_image)
    {
        int f = 4;
        cv::resize(cv_image->image, cv_image->image, cv_image->image.size() / f);
        // cv::imshow("original", cv_image->image);
        // cv::waitKey(1);
        cv_image->image = saturation(cv_image->image, 100);                       // 饱和度调整参数，-100 — 100, 正数饱和度增强，负数饱和度减弱
        cv::GaussianBlur(cv_image->image, cv_image->image, cv::Size(3, 3), 0, 0); // 滤波预处理
        std::vector<cv::Vec2f> lines;
        LBD(cv_image->image, lines);
        cv::Mat b = cv_image->image.clone();
        border[0] = border[1] = 0;
        if (lines.empty())
        {
            ROS_WARN("Could not find border!");
            return false;
        }
        int cnt = 0;
        for (cv::Vec2f &line : lines)
        {
            double rho = line[0], theta = line[1];
            double theta_d = Angle::degree(theta);
            if (abs(theta_d - 90) < theta_thr_horizontal_)
            {
                border[0] += rho;
                border[1] += theta;
                cnt++;
                if (show_detection)
                {
                    // 绘制蓝色
                    plot_line(cv_image->image, rho, theta, cv::Scalar(255, 0, 0));
                }
            }
            else if (show_detection)
            {
                // 排除的直线绘制绿色
                plot_line(cv_image->image, rho, theta, cv::Scalar(0, 255, 0));
            }
        }
        if (show_detection && !cv_image->image.empty())
        {
            // cv::imshow("Hough", cv_image->image);
            plot_line(b, border[0], border[1], cv::Scalar(0, 0, 255));
            // cv::waitKey(1);
            // cv::imshow("border", b);
            debug_image = cv_image->toImageMsg();
        }
        if (!cnt)
        {
            ROS_WARN("Could not find border!");
            return false;
        }
        border[0] /= cnt;
        border[1] /= cnt;
        // 对之前resize的恢复
        border[0] *= f;
        return true;
    }

    void BorderMethod::LBD_thershold_func(cv::Mat &enhanced, std::vector<cv::Vec2f> &lines, int threshold)
    {
        using namespace cv;
        Mat gray;
        cvtColor(enhanced, gray, COLOR_BGR2GRAY);
        imshow("Gray", gray);
        Mat ThreImg;
        adaptiveThreshold(gray, ThreImg, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, -1);
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(ThreImg, ThreImg, MORPH_CLOSE, element, Point(-1, -1), 3);
        imshow("Thresh", ThreImg);

        Canny(enhanced, ThreImg, 50, 200, 3);
        imshow("edge", ThreImg);

        // Mat LineImg;
        // cvtColor(ThreImg, LineImg, COLOR_GRAY2BGR);
        HoughLines(ThreImg, lines, 1, CV_PI / 180, threshold, 0, 0);
    }

    void BorderMethod::LBD_color_func(cv::Mat &enhanced, std::vector<cv::Vec2f> &lines, int threshold)
    {
        using namespace cv;
        Mat HSVImg;
        cvtColor(enhanced, HSVImg, COLOR_BGR2HSV);
        imshow("HSVImg", HSVImg);

        Scalar Boundary_low = Scalar(78, 0, 0);
        Scalar Boundary_high = Scalar(179, 255, 255);
        Mat RangeImg;
        inRange(HSVImg, Boundary_low, Boundary_high, RangeImg);
        imshow("RangeImg", RangeImg);

        Mat edgeImg;
        Canny(RangeImg, edgeImg, 50, 200, 3);
        imshow("edge", edgeImg);

        // Mat LineImg;
        // cvtColor(edgeImg, LineImg, COLOR_GRAY2BGR);
        HoughLines(edgeImg, lines, 1, CV_PI / 180, threshold, 0, 0);
    }
} // namespace my_hand_eye
