#ifndef _QRCODE_DETECTOR_H_
#define _QRCODE_DETECTOR_H_
#include <std_msgs/Int16MultiArray.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"

namespace eye
{
    class QRcodeDetector : public nodelet::Nodelet
    {
    private:
        cv::RNG rngs_ = { 12345 }; 
        cv_bridge::CvImagePtr cv_image_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber camera_image_subscriber_;
    public:
        QRcodeDetector() = default;
        ~QRcodeDetector() = default;
        void onInit();
        void imageCallback(const sensor_msgs::ImageConstPtr& image_rect);
    };  

}

#include "opencv2/opencv.hpp"
#endif // !_QRCODE_DETECTOR_H_