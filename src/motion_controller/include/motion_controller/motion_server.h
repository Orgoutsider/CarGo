#ifndef _MOTION_SERVER_H_
#define _MOTION_SERVER_H_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <motion_controller/MoveAction.h>
#include <motion_controller/TwistMightEnd.h>

#include "motion_controller/pid_controller.h"

namespace motion_controller
{
    typedef actionlib::SimpleActionServer<MoveAction> Server;

    class MotionServer
    {
    private:
        double kp_angular_;
        double ki_angular_;
        double kd_angular_;
        double kp_linear_;
        double ki_linear_;
        double kd_linear_;
        Server server_;
        tf2_ros::Buffer buffer_;              // 创建一个缓冲。
        tf2_ros::TransformListener listener_; // 用刚创建的缓冲Buffer来初始化创建一个TransformListener类的对象用于守听Transform消息。
        geometry_msgs::PoseStamped pose_goal_;
        ros::Publisher cmd_vel_publisher_; // 速度话题发布者
        // pid速度参考：线速度0.2，角速度0.5
        // PIDController pid_;
        // Optional callback that gets called in a separate thread whenever a new goal is received, allowing users to have blocking callbacks.
        void _execute_callback(const motion_controller::MoveGoalConstPtr &goal);
        // 挤占服务时回调函数
        void _preempt_callback();
        // 将目标点转换到odom_combined坐标系并存储
        bool _add_pose_goal(geometry_msgs::Pose2D pose);
        bool _get_pose_now(geometry_msgs::Pose2D &pose, ros::Time &now);

    public:
        MotionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    };

} // namespace motion_controller

#endif // !_MOTION_SERVER_H_