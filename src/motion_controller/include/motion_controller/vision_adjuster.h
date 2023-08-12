#ifndef _VISION_ADJUSTER_H_
#define _VISION_ADJUSTER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
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
        enum Direction
        {
            direction_void,
            direction_theta,
            direction_x,
            direction_y
        };

        bool debug_;
        Direction changing_;
        Direction unchanging_;
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
        double threshold_angular_;
        double threshold_linear_;
        PIDController pid_;
        // 机械手摄像头视觉信息订阅
        ros::Publisher cmd_vel_publisher_;
        ros::Timer timer_;                    // 定时器利用odom信息
        std::string target_frame_;            // 目标坐标系
        tf2_ros::Buffer buffer_;              // 创建一个缓冲
        tf2_ros::TransformListener listener_; // 用刚创建的缓冲Buffer来初始化创建一个TransformListener类的对象用于守听Transform消息。
        message_filters::Subscriber<my_hand_eye::Pose2DMightEnd> eye_subscriber_;
        tf2_ros::MessageFilter<my_hand_eye::Pose2DMightEnd> tf2_filter_;
        // 设置在运动中静止的坐标
        geometry_msgs::PoseStamped pose_goal_;
        // 动态参数
        dynamic_reconfigure::Server<params_PID_visionConfig> dr_server_;
        void _eye_callback(const my_hand_eye::Pose2DMightEndConstPtr &msg);
        void _timer_callback(const ros::TimerEvent &event);
        void _dr_callback(params_PID_visionConfig &config, uint32_t level);
        bool _add_pose_goal(const my_hand_eye::Pose2DMightEnd &pose);
        bool _get_pose_now(geometry_msgs::Pose2D &pose, ros::Time &stamp);

    public:
        VisionAdjuster();
    };

} // namespace motion_controller

#endif // !_VISION_ADJUSTER_H_
