#ifndef _MY_EYE_H_
#define _MY_EYE_H_

#include "my_hand_eye/arm_controller.h"
#include <actionlib/server/simple_action_server.h>
// #include <dynamic_reconfigure/server.h>
#include <my_hand_eye/ArrayofTaskArrays.h>
#include <my_hand_eye/ArmAction.h>
// #include <my_hand_eye/drConfig.h>


namespace my_hand_eye
{
    typedef actionlib::SimpleActionServer<my_hand_eye::ArmAction> Server;

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
        Server as_;
        // dynamic_reconfigure::Server<my_hand_eye::drConfig> dr_server_;
        void task_callback(const my_hand_eye::ArrayofTaskArraysConstPtr &task);
        void image_callback(const sensor_msgs::ImageConstPtr &image_rect);
        // Optional callback that gets called in a separate thread whenever a new goal is received, allowing users to have blocking callbacks.
        void execute_callback(const ArmGoalConstPtr &goal);
        // 挤占服务时回调函数
        void preempt_callback();
        // 动态参数回调函数
        // void dr_callback(drConfig& config, uint32_t level);
    public:
        MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    };

} // namespace my_hand_eye

#endif // !_MY_EYE_H_