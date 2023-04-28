#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <motion_controller/status.h>

#include "opencv2/opencv.hpp"

using namespace cv;
RNG rng_(12345);
Scalar BK_low__ = Scalar(0, 0, 0);
Scalar BK_up__ = Scalar(180, 255, 100); // 夜晚80左右，下午100
class Judge
{
private:
    cv_bridge::CvImagePtr cv_image_;
    int cnt, judge_cnt_;
    const int fault_tolerance_ = 130; // 由于干扰，可能存在误判
    const double goal_y_ = 66;        // 目标位置0-75
    bool flag_ = false;               // 是否到达制定位置
public:
    Judge() : cnt(0), judge_cnt_(0){};
    ros::Publisher judge_pub_;
    void imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
    {
        if (cnt < 100)
        {
            cnt++;
            // ROS_INFO("wait... ");
            return;
        }
        try
        {
            cv_image_ = cv_bridge::toCvCopy(image_rect, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if (!(cv_image_->image.data))
        {
            ROS_ERROR("No data!");
            return;
        }
        Mat srcF;
        resize(cv_image_->image, srcF, Size(320, 240));
        srcF = srcF(Range(145, 220), Range(60, 260));
        // 色彩分离查找车道线
        cvtColor(srcF, srcF, COLOR_BGR2HSV);
        inRange(srcF, BK_low__, BK_up__, srcF);
        // 可能需要去除干扰轮廓,(5, 3)可能更好一点
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 1));
        // morphologyEx(srcF, srcF, MORPH_CLOSE, element);
        dilate(srcF, srcF, element);
        // imshow("line", srcF);
        // waitKey(1);
        Canny(srcF, srcF, 50, 100, 3);
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, 50, 0, 0); // 根据实际调整，夜晚40，下午50
        // Mat res = Mat::zeros(srcF.size(), CV_8UC3);
        int tot = 0, sum_y = 0;
        if (lines.size())
        {
            for (int i = 0; i < lines.size(); i++)
            {
                double rho = lines[i][0], theta = lines[i][1];
                double a = cos(theta), b = sin(theta);
                // Point pt1, pt2;
                // double x0 = a * rho, y0 = b * rho;
                // pt1.x = cvRound(x0 + 1000 * (-b));
                // pt1.y = cvRound(y0 + 1000 * (a));
                // pt2.x = cvRound(x0 - 1000 * (-b));
                // pt2.y = cvRound(y0 - 1000 * (a));
                // Scalar color = Scalar(rng_.uniform(0, 255), rng_.uniform(0, 255), rng_.uniform(0, 255));
                // line(res, pt1, pt2, color, 1, LINE_AA);
                // imshow("Turning_line", res);
                // waitKey(1);

                // 计算斜率
                double theta_d = theta * 180 / CV_PI;
                if (judge_cnt_ < fault_tolerance_ && !flag_)
                {
                    motion_controller::status status;
                    status.err = 0;
                    status.judge = status.NORMAL;
                    judge_pub_.publish(status);
                }
                if (87 < theta_d && theta_d < 93 && !flag_)
                {
                    // ROS_INFO_STREAM("theta_d:" << theta_d << " cnt:" << judge_cnt_);
                    if (judge_cnt_ < fault_tolerance_)
                    {
                        judge_cnt_++;
                        continue;
                    }
                    else
                    {
                        tot++;
                        sum_y += cvRound((rho - a * srcF.cols / 2) / b);
                    }
                }
            }
            if (judge_cnt_ >= fault_tolerance_ && tot != 0 && !flag_)
            {
                double now_y = sum_y * 1.0 / tot;
                motion_controller::status status;
                ROS_INFO_STREAM(now_y);
                if ((goal_y_ - now_y) * (goal_y_ - now_y) < 1)
                {
                    status.err = 0;
                    status.judge = status.CORNER;
                    judge_pub_.publish(status);
                    ROS_INFO("Turning...");
                    flag_ = true;
                }
                else
                {
                    status.err = goal_y_ - now_y;
                    status.judge = status.ADJUSTING;
                    judge_pub_.publish(status);
                }
            }
            if (flag_)
            {
                flag_ = false;
                judge_cnt_ = 0;
                cnt = -400;
            }
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "corners_judge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    image_transport::ImageTransport it(nh);
    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");
    Judge judge;
    judge.judge_pub_ = nh.advertise<motion_controller::status>("corners_judge", 1);
    image_transport::Subscriber camera_image_subscriber =
        it.subscribe("/usb_cam/image_rect_color", 1, &Judge::imageCallback, &judge, image_transport::TransportHints(transport_hint));
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}