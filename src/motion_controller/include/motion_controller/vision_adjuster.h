#ifndef _VISION_ADJUSTER_H_
#define _VISION_ADJUSTER_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <motion_controller/TwistMightEnd.h>

#include "motion_controller/pid_controller.h"

namespace motion_controller
{
    class VisionAdjuster
    {
    private:
        // 弯道pid
        double kp_usb_cam_;
        double ki_usb_cam_;
        double kd_usb_cam_;
        // 角位置pid
        double kp_eye_angular_;
        double ki_eye_angular_;
        double kd_eye_angular_;
        // 坐标位置pid
        double kp_eye_linear_;
        double ki_eye_linear_;
        double kd_eye_linear_;
        // 车头摄像头视觉信息订阅
        ros::Subscriber usb_cam_subscriber_;
        // 车头摄像头pid
        // PIDController pid_usb_cam_;
        // 机械手摄像头视觉信息订阅
        ros::Subscriber eye_subscriber_;
        // 机械手摄像头pid
        // PIDController pid_eye_;
        ros::Publisher cmd_vel_publisher_;
        void _usb_cam_callback(const std_msgs::Float64ConstPtr &msg);
        void _eye_callback(const geometry_msgs::Pose2DConstPtr &msg);
    public:
        VisionAdjuster();
    };
        
} // namespace motion_controller


#endif // !_VISION_ADJUSTER_H_
