#ifndef _VISION_ADJUSTER_H_
#define _VISION_ADJUSTER_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <motion_controller/Pose2DMightEnd.h>
#include <motion_controller/Distance.h>
#include <motion_controller/TwistMightEnd.h>
#include <motion_controller/params_PID_visionConfig.h>

#include "motion_controller/pid_controller.h"

namespace motion_controller
{
    enum
    {
        level_usb_cam,
        level_eye
    };

    class VisionAdjuster
    {
    private:
        bool param_modification_;
        int level_; // 优先级，优先使用机械手摄像头视觉信息
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
        // 机械手摄像头视觉信息订阅
        ros::Subscriber eye_subscriber_;
        ros::Publisher cmd_vel_publisher_;
        // 动态参数
        dynamic_reconfigure::Server<params_PID_visionConfig> dr_server_;
        void _usb_cam_callback(const DistanceConstPtr &msg);
        void _eye_callback(const Pose2DMightEndConstPtr &msg);
        void _dr_callback(params_PID_visionConfig &config, uint32_t level);

    public:
        VisionAdjuster();
    };

} // namespace motion_controller

#endif // !_VISION_ADJUSTER_H_
