#ifndef _QRCODE_DETECTOR_H_
#define _QRCODE_DETECTOR_H_

#define FRAME_HEADER 0X7B	 // Frame head //帧头
#define FRAME_TAIL 0X7D		 // Frame tail //帧尾

#include <ros/ros.h>
#include <serial/serial.h>
// #include <boost/thread/lock_guard.hpp>
// #include <boost/thread/thread.hpp>
// #include <boost/thread/mutex.hpp>

namespace my_hand_eye
{
    class QRcodeDetector : public nodelet::Nodelet
    {
    private:
        // boost::mutex mtx_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber zxing_subscriber_;
        ros::Subscriber zbar_subscriber_;
        ros::Publisher QR_code_publisher_;
        ros::Timer esp32_timer_;
        // 声明串口对象
        serial::Serial esp32_serial_;
        bool flag_; // 是否已检测到二维码
        // 文字坐标，假定显示文字的图像为1080p，已预设好
        const int ptx_info[6] = {350, 800, 1240, 350, 800, 1240};
        const int pty_info[6] = {430, 430, 430, 950, 950, 950};
        // 文字坐标偏移量，前期调试用
        const int txt_Xoffset = 0;
        const int txt_Yoffset = 0;
        // 文字大小
        const int txt_size = 30;
        // 文字厚度
        const int txt_thick = 50;
    public:
        QRcodeDetector() = default;
        ~QRcodeDetector() = default;
        void onInit();
        void zxingCallback(const zxing_msgs::QRCodeArrayConstPtr &msgs);
        void zbarCallback(const std_msgs::StringConstPtr &msgs);
        void connectCallback();
        void disconnectCallback();
        void esp32Callback(const ros::TimerEvent &event);
        bool checkString(const std::string &str);
        void screenShow(cv::Mat &img);
    };
}
#endif // !_QRCODE_DETECTOR_H_