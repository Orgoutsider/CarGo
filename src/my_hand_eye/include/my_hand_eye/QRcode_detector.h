#ifndef _QRCODE_DETECTOR_H_
#define _QRCODE_DETECTOR_H_
#include "opencv2/opencv.hpp"

#include "my_hand_eye/pose.hpp"

#include <std_msgs/Int16MultiArray.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <my_hand_eye/ArrayofTaskArrays.h>


namespace my_hand_eye
{
    class QRcodeDetector : public nodelet::Nodelet
    {
    private:
        ros::Subscriber QR_code_subscriber_;
        ros::Publisher QR_code_publisher_;
        ros::Publisher debug_image_publisher_;
        my_hand_eye::ArmController arm_controller_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber camera_image_subscriber_;
        bool flag_;// 是否已检测到二维码
        //文字坐标，假定显示文字的图像为1080p，已预设好
        int ptx_info[6] = { 350,800,1240,350,800,1240 };
        int pty_info[6] = { 430,430,430,950,950,950 };
        //文字坐标偏移量，前期调试用
        const int txt_Xoffset = 0;
        const int txt_Yoffset = 0;
        //文字大小
        const int txt_size = 30;
        //文字厚度
        const int txt_thick = 50;
        const double z_floor = 2.9;//底盘距地6mm，物块高度一半35mm
    public:
        QRcodeDetector() = default;
        ~QRcodeDetector() = default;
        void onInit();
        void Callback(const std_msgs::StringConstPtr &info);
        void imageCallback(const sensor_msgs::ImageConstPtr& image_rect);
    };  

}
#endif // !_QRCODE_DETECTOR_H_