#ifndef _COLORFINDING_H_
#define _COLORFINDING_H_
#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace eye
{
    class ColorDetector : public nodelet::Nodelet
    {
    private:
        std_msgs::Int16 task;
        const cv::Scalar R_Low = cv::Scalar(156, 43, 46);
        const cv::Scalar R_up = cv::Scalar(180, 255, 255);

        const cv::Scalar G_Low = cv::Scalar(30, 43, 46);
        const cv::Scalar G_up = cv::Scalar(80, 255, 255);

        const cv::Scalar B_Low = cv::Scalar(90, 43, 80);
        const cv::Scalar B_up = cv::Scalar(124, 255, 255);

        const cv::Scalar Low[3] = { B_Low, G_Low, R_Low };
        const cv::Scalar Up[3] = { B_up, G_up, R_up };
        cv_bridge::CvImagePtr cv_image_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber camera_image_subscriber_;
    public:
        ColorDetector() = default;
        ~ColorDetector() = default;
        const unsigned blue = 0, green = 1, red = 2;//这里不太清楚
        void onInit();
        void imageCallback(const sensor_msgs::ImageConstPtr& image_rect);
        cv::Mat find_color(unsigned RGB);
    };    
} // namespace eye
#endif // !_COLORFINDING_H_