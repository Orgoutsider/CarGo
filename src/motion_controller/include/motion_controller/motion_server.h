#ifndef _MOTION_SERVER_H_
#define _MOTION_SERVER_H_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_controller/MoveAction.h>
#include <dynamic_reconfigure/server.h>
#include <motion_controller/params_PID_srvConfig.h>

#include "motion_controller/pid_controller.h"

namespace motion_controller
{
    typedef actionlib::SimpleActionServer<MoveAction> Server;

    class MotionServer
    {
    private:
        bool debug_; // 是否进行pid调节
        double kp_angular_[2];
        double ki_angular_[2];
        double kd_angular_[2];
        double kp_linear_[2];
        double ki_linear_[2];
        double kd_linear_[2];
        Server server_;
        tf2_ros::Buffer buffer_;              // 创建一个缓冲。
        tf2_ros::TransformListener listener_; // 用刚创建的缓冲Buffer来初始化创建一个TransformListener类的对象用于守听Transform消息。
        std_msgs::Header header_;
        geometry_msgs::PoseStamped pose_goal_;
        ros::Publisher cmd_vel_publisher_;                            // 速度话题发布者
        dynamic_reconfigure::Server<params_PID_srvConfig> dr_server_; // 创建动态参数服务器对象
        // pid速度参考：线速度0.2，角速度0.5
        // Optional callback that gets called in a separate thread whenever a new goal is received, allowing users to have blocking callbacks.
        void _execute_callback(const motion_controller::MoveGoalConstPtr &goal);
        void _dr_callback(motion_controller::params_PID_srvConfig &config, uint32_t level);
        // 将目标点转换到odom_combined坐标系并存储
        bool _add_pose_goal(const geometry_msgs::Pose2D &pose);
        // 获取当前位姿，如果获取失败，则不改变pose
        bool _get_pose_now(geometry_msgs::Pose2D &pose);

    public:
        MotionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    };

} // namespace motion_controller

#endif // !_MOTION_SERVER_H_