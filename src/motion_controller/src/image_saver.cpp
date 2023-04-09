#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
{
    cv_bridge::CvImagePtr cv_image;
    try
    {
        cv_image = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
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
    cv::imwrite("/home/nano/car/car/car_ws/src/motion_controller/img/1.png", cv_image->image);
    ROS_INFO("Succeeded to save!");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_saver");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    image_transport::ImageTransport it(nh);
    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");
    image_transport::Subscriber camera_image_subscriber =
        it.subscribe("/usb_cam/image_rect_color", 1, imageCallback, image_transport::TransportHints(transport_hint));
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}