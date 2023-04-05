#ifndef _MY_EYE_H_
#define _MY_EYE_H_

#include "my_hand_eye/arm_controller.h"

#include <my_hand_eye/ArrayofTaskArrays.h>

namespace my_hand_eye
{
    class MyEye
    {
    private:
        ros::Publisher debug_image_publisher_;
        ArmController arm_controller_;
        ArrayofTaskArraysConstPtr tasks_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        std::string transport_hint_;
        image_transport::Subscriber camera_image_subscriber_;
        ros::Subscriber task_subscriber_;
    public:
        MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        void task_callback(const my_hand_eye::ArrayofTaskArraysConstPtr &task);
        void image_callback(const sensor_msgs::ImageConstPtr &image_rect);
    };

} // namespace my_hand_eye

#endif // !_MY_EYE_H_