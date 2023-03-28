#ifndef _MY_EYE_H_
#define _MY_EYE_H_

#include "my_hand_eye/arm_controller.h"

namespace my_hand_eye
{
    class MyEye
    {
    private:
        ros::Publisher debug_image_publisher_;
        ArmController arm_controller_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber camera_image_subscriber_;
    public:
        MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        void image_callback(const sensor_msgs::ImageConstPtr& image_rect);
    };

} // namespace my_hand_eye

#endif // !_MY_EYE_H_