#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"

using namespace cv; //

Scalar BK_low = Scalar(0, 0, 0);
Scalar BK_up = Scalar(180, 255, 100); // 夜晚80左右，下午100

class Driver
{
private:
    cv_bridge::CvImagePtr cv_image_;
    geometry_msgs::Twist twist_;
    const int judge = 20; // 0-100
    const double Kp = 0.0015, Kd = 0.001;

    int last_err, tmp, cnt;

public:
    Driver() : tmp(0), last_err(0), cnt(0){};
    ros::Publisher cmd_vel_pub_;
    void imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
    {
        if (cnt < 10) // 等待摄像头适应光线
        {
            cnt++;
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
        srcF = srcF(Range(130, 200), Range(20, 300));

        // ROS_INFO_STREAM("c:" << srcF.channels());
        // srcF.convertTo(srcF, CV_8UC3);
        cvtColor(srcF, srcF, COLOR_BGR2HSV);
        inRange(srcF, BK_low, BK_up, srcF);
        // threshold(srcF, srcF, 100, 255, THRESH_BINARY);
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 1));
        // morphologyEx(srcF, srcF, MORPH_CLOSE, element);
        dilate(srcF, srcF, element);
        Canny(srcF, srcF, 50, 100, 3);
        // imshow("line", srcF);
        // waitKey(1);
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, 50, 0, 0); // 根据实际调整，夜晚40，下午50
        Mat res(srcF.size(), CV_8UC3, Scalar::all(0));     // int flag1 = 0;
        int minx, maxx, nowx, last_maxx, last_minx, bar = 400;
        // int err_t = 0;
        if (lines.size() > 1)
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
                // line(res, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                // imshow("line1", res);
                // waitKey(1);
                nowx = cvRound((rho - b * judge) / a);
                if (i == 0)
                {
                    minx = maxx = nowx;
                }

                minx = (nowx < minx) ? nowx : minx;
                maxx = (nowx > maxx) ? nowx : maxx;
                // if (i != 0)
                // {
                //     minx = ((minx - last_minx) * (minx - last_minx) > bar) ? last_minx : minx;
                //     maxx = ((maxx - last_maxx) * (maxx - last_maxx) > bar) ? last_maxx : maxx;
                // }
                // last_minx = minx;
                // last_maxx = maxx;
            }
            int err = (minx + maxx) / 2 - srcF.cols / 2;
            err = (err < -40 || err > 40) ? last_err : err;
            if (tmp = 0)
            {
                tmp = 1;
                last_err = err;
            }
            int d_err = err - last_err;
            // ROS_INFO_STREAM("err" << err << " size:" << lines.size());
            twist_.linear.x = 0.1;
            twist_.angular.z = (err != 0) ? -Kp * err - Kd * d_err : 0;
            last_err = err;
            // ROS_INFO_STREAM(minx << ' ' << maxx);
        }
        else
        {
            twist_.linear.x = 0;
            twist_.angular.z = 0;
        }
        // ROS_INFO_STREAM(lines.size());
        cmd_vel_pub_.publish(twist_);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    image_transport::ImageTransport it(nh);
    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");
    Driver driver;
    driver.cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_ori", 1);
    image_transport::Subscriber camera_image_subscriber =
        it.subscribe("/usb_cam/image_raw", 1, &Driver::imageCallback, &driver, image_transport::TransportHints(transport_hint));
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}