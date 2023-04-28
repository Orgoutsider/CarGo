#include "motion_controller/motion_controller.h"

namespace motion_controller
{
    MotionController::MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : r_start_(100), r_end_(160),
          c_start_(20), c_end_(width_ - c_start_), threshold_(50),
          black_low_(0, 0, 0), black_up_(180, 255, 100), client_(nh, "Move", true)
    {
        pnh.param<bool>("param_modification", param_modification_, false);
        it_ = std::shared_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(nh));
        std::string transport_hint;
        pnh.param<std::string>("transport_hint", transport_hint, "raw");
        image_subscriber_ = it_->subscribe("/usb_cam/image_rect_color", 1, &MotionController::_image_callback, this, image_transport::TransportHints(transport_hint));
        vision_publisher = nh.advertise<std_msgs::Float64>("/vision_motion", 5);
    }

    bool MotionController::_turn(bool left)
    {
        if (!client_.isServerConnected())
            client_.waitForServer();
    }

    void MotionController::_image_callback(const sensor_msgs::ImageConstPtr &image_rect)
    {
        using namespace cv;
        static int cnt = 0;
        cv_bridge::CvImagePtr cv_image;
        try
        {
            cv_image = cv_bridge::toCvCopy(image_rect, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if (!(cv_image->image.data))
        {
            ROS_ERROR("No data!");
            return;
        }
        Mat srcF;
        resize(cv_image->image, srcF, Size(width_, height_));
        srcF = srcF(Range(r_start_, r_end_), Range(c_start_, c_end_));
        // 色彩分离查找车道线
        cvtColor(srcF, srcF, COLOR_BGR2HSV);
        inRange(srcF, black_low_, black_up_, srcF);
        // 可能需要去除干扰轮廓,(5, 3)可能更好一点
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 1));
        // morphologyEx(srcF, srcF, MORPH_CLOSE, element);
        dilate(srcF, srcF, element);
        if (param_modification_)
        {
            imshow("line", srcF);
            waitKey(1);
        }
        Canny(srcF, srcF, 50, 100, 3);
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, threshold_, 0, 0); // 根据实际调整，夜晚40，下午50
        int tot = 0, y_sum = 0;
        if (lines.size())
        {
            Mat res = Mat::zeros(srcF.size(), CV_8UC3);
            for (int i = 0; i < lines.size(); i++)
            {
                double rho = lines[i][0], theta = lines[i][1];
                double a = cos(theta), b = sin(theta);

                // 计算斜率
                double theta_d = theta * 180 / CV_PI;
                if (87 < theta_d && theta_d < 93)
                {
                    // ROS_INFO_STREAM("theta_d:" << theta_d << " cnt:" << judge_cnt_);
                    if (cnt < cnt_tolerance_)
                    {
                        cnt++;
                        continue;
                    }
                    else
                    {
                        tot++;
                        y_sum += cvRound((rho - a * srcF.cols / 2) / b);
                    }
                }
                else  
                    cnt = 0;
                if (param_modification_)
                {
                    Point pt1, pt2;
                    double x0 = a * rho, y0 = b * rho;
                    pt1.x = cvRound(x0 + 1000 * (-b));
                    pt1.y = cvRound(y0 + 1000 * (a));
                    pt2.x = cvRound(x0 - 1000 * (-b));
                    pt2.y = cvRound(y0 - 1000 * (a));
                    line(res, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                    line(res, Point(0, y_ground_), Point(c_end_ - c_start_, y_ground_), Scalar(255, 0, 0), 1, LINE_AA);
                    imshow("res", res);
                    waitKey(1);
                }
            }
            if (cnt >= cnt_tolerance_ && tot != 0)
            {
                double y_now = y_sum * 1.0 / tot;
                // ROS_INFO_STREAM(y_now);
                double distance;
                if (y_ground_ < y_now - 0.01)
                    distance = 1 / (1 / y_ground_ - 1 / y_now);
                else
                {
                    ROS_WARN("y_ground is invalid!");
                    return;
                }
                if (distance < distance_thr_)
                {
                    cnt = 0;
                    vision_publisher.publish(std_msgs::Float64());
                    // 客户端转弯
                }
                else
                {
                    std_msgs::Float64 msg;
                    msg.data = distance;
                    vision_publisher.publish(msg);
                }
            }
        }
    }

} // namespace motion_controller
