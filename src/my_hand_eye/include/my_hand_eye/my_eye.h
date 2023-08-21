#ifndef _MY_EYE_H_
#define _MY_EYE_H_

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <my_hand_eye/ArrayofTaskArrays.h>
#include <my_hand_eye/ArmAction.h>
#include <my_hand_eye/drConfig.h>

#include "my_hand_eye/arm_controller.h"

namespace my_hand_eye
{
    typedef actionlib::SimpleActionServer<my_hand_eye::ArmAction> Server;

    class MyEye
    {
    private:
        bool debug_;
        bool finish_adjusting_; // 反馈：是否已完成位姿调整
        bool finish_;           // 是否已完成arm任务
        int task_idx_;          // 第几种颜色
        ros::Publisher debug_image_publisher_;
        ArmController arm_controller_;
        ArrayofTaskArrays tasks_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        std::string transport_hint_;
        image_transport::Subscriber camera_image_subscriber_;
        ros::Subscriber task_subscriber_;
        ros::Publisher pose_publisher_;
        Server as_;
        ArmGoal arm_goal_;
        dynamic_reconfigure::Server<my_hand_eye::drConfig> dr_server_;
        void next_task();
        Color which_color(bool next = false) const;
        void task_callback(const my_hand_eye::ArrayofTaskArraysConstPtr &task);
        void image_callback(const sensor_msgs::ImageConstPtr &image_rect);
        // Optional callback that gets called in a separate thread whenever a new goal is received, allowing users to have blocking callbacks.
        void goal_callback();
        // 挤占服务时回调函数
        void preempt_callback();
        // 强制关闭与底盘的联系
        void cancel_all();
        // 转盘处执行的函数
        bool operate_center(const sensor_msgs::ImageConstPtr &image_rect,
                            sensor_msgs::ImagePtr &debug_image);
        // 椭圆处执行的函数
        bool operate_ellipse(const sensor_msgs::ImageConstPtr &image_rect,
                             sensor_msgs::ImagePtr &debug_image);
        // 边界处执行的函数
        bool operate_border(const sensor_msgs::ImageConstPtr &image_rect,
                            sensor_msgs::ImagePtr &debug_image);
        // 动态参数回调函数
        void dr_callback(drConfig &config, uint32_t level);

    public:
        MyEye(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    };

} // namespace my_hand_eye

#endif // !_MY_EYE_H_