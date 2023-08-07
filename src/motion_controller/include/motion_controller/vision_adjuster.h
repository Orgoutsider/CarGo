#ifndef _VISION_ADJUSTER_H_
#define _VISION_ADJUSTER_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <my_hand_eye/Pose2DMightEnd.h>
#include <motion_controller/params_PID_visionConfig.h>

#include "motion_controller/pid_controller.h"

namespace motion_controller
{
    class VisionAdjuster
    {
    private:
        enum Unchanging
        {
            not_change_x,
            not_change_y,
            not_change_theta
        };

        bool debug_;
        Unchanging not_change_;
        // // 弯道pid
        // double kp_usb_cam_;
        // double ki_usb_cam_;
        // double kd_usb_cam_;
        // 角位置pid
        double kp_eye_angular_;
        double ki_eye_angular_;
        double kd_eye_angular_;
        // 坐标位置pid
        double kp_eye_linear_;
        double ki_eye_linear_;
        double kd_eye_linear_;
        PIDController eye_pid_;
        // 车头摄像头视觉信息订阅
        ros::Subscriber usb_cam_subscriber_;
        // 机械手摄像头视觉信息订阅
        ros::Subscriber eye_subscriber_;
        ros::Publisher cmd_vel_publisher_;
        tf2_ros::Buffer buffer_;              // 创建一个缓冲。
        tf2_ros::TransformListener listener_; // 用刚创建的缓冲Buffer来初始化创建一个TransformListener类的对象用于守听Transform消息。
        // 设置在运动中静止的坐标
        geometry_msgs::TransformStamped tfs_;
        // 动态参数
        dynamic_reconfigure::Server<params_PID_visionConfig> dr_server_;
        void _eye_callback(const my_hand_eye::Pose2DMightEndConstPtr &msg);
        void _dr_callback(params_PID_visionConfig &config, uint32_t level);
        bool _get_transform();
        bool _get_change(double &change);

    public:
        VisionAdjuster();
    };

} // namespace motion_controller

#endif // !_VISION_ADJUSTER_H_
