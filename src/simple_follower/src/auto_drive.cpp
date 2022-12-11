#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"

using namespace cv;

class Driver
{
private:
    cv_bridge::CvImagePtr cv_image_;
    geometry_msgs::Twist twist_;
    const int judge = 0;
    const double Kp = 0.005, Kd = 0.001;
    Scalar BK_low = Scalar(0, 0, 0);
    Scalar BK_up = Scalar(180, 255, 46);
    int last_err;
    ros::Publisher cmd_vel_pub_;
public:
    Driver (ros::NodeHandle &nh)
    {
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_ori", 1);
    }
    void imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
    {
        try
        {
            cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
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
        resize(cv_image_->image, srcF, Size(640, 480));
        srcF = srcF(Range(100, 270), Range(0, 480)); // 根据实际调整
        ROS_INFO_STREAM("c:" << srcF.channels());
        cvtColor(srcF, srcF, COLOR_BGR2HSV);
        inRange(srcF, BK_low, BK_up, srcF);
        //threshold(srcF, srcF, 100, 255, THRESH_BINARY);
        Canny(srcF, srcF, 50, 100, 3);
        std::vector<Vec2f> lines;
        HoughLines(srcF, lines, 1, CV_PI / 180, 50, 0, 0); // 根据实际调整
        imshow("line1", srcF);
        Mat res(srcF.size(), CV_8UC3, Scalar::all(0)); // int flag1 = 0;
        int minx, maxx, nowx, last_maxx, last_minx, bar = 400;
        // int err_t = 0;
        if (lines.size() > 2)
        {
            for (int i = 0; i < lines.size(); i++)
            {

                double rho = lines[i][0], theta = lines[i][1];
                double a = cos(theta), b = sin(theta);
                // cout << i << "rho: " << rho << endl;
                // cout << i << "theta: " << theta << endl;
                // Point pt1, pt2;
                // double x0 = a * rho, y0 = b * rho;
                // pt1.x = cvRound(x0 + 1000 * (-b));
                // pt1.y = cvRound(y0 + 1000 * (a));
                // pt2.x = cvRound(x0 - 1000 * (-b));
                // pt2.y = cvRound(y0 - 1000 * (a));
                // Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                // line(res, pt1, pt2, color, 1, LINE_AA);
                // line(res, Point(0, 0), Point(479, 0), Scalar(0, 0, 255), 3, LINE_AA);
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
            int d_err = err - last_err;
            ROS_INFO_STREAM("err" << err);
            twist_.linear.x = 0.15;
            twist_.angular.z = (err != 0) ? -Kp * err - Kd * d_err : 0;

            // cout << minx << ' ' << maxx << endl;
            // cout << err << endl;
        }
        else
        {
            twist_.linear.x -= 0.01;
            twist_.angular.z = 0;
        }
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
    Driver driver(nh);
    image_transport::Subscriber camera_image_subscriber =
        it.subscribe("/usb_cam/image_raw", 1, &Driver::imageCallback, &driver, image_transport::TransportHints(transport_hint));
    ros::spin();
    return 0;
}