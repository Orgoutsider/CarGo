#include <motion_controller/TwistMightEnd.h>

#include "motion_controller/vision_adjuster.h"

namespace motion_controller
{
    VisionAdjuster::VisionAdjuster()
        : listener_(buffer_), target_frame_("odom_combined"),
          tf2_filter_(eye_subscriber_, buffer_, target_frame_, 15, 0),
          unchanging_(direction_void), changing_(direction_theta),
          kp_eye_angular_(1.9), ki_eye_angular_(0.2), kd_eye_angular_(0.0),
          kp_eye_linear_(1.0), ki_eye_linear_(0.0), kd_eye_linear_(0.4),
          threshold_angular_(0.015), threshold_linear_x_(0.01), threshold_linear_y_(0.005),
          pid_({0}, {kp_eye_angular_},
               {ki_eye_angular_}, {kd_eye_angular_},
               {threshold_angular_}, {0.05}, {0.4})
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        eye_subscriber_.subscribe(nh, "/vision_eye", 3);
        tf2_filter_.registerCallback(boost::bind(&VisionAdjuster::_eye_callback, this, _1));
        cmd_vel_publisher_ = nh.advertise<TwistMightEnd>("/cmd_vel_vision", 3);
        timer_ = nh.createTimer(ros::Rate(10), &VisionAdjuster::_timer_callback, this, false, false);
        pnh.param<bool>("debug", debug_, false);
        if (debug_)
            dr_server_.setCallback(boost::bind(&VisionAdjuster::_dr_callback, this, _1, _2));
    }

    void VisionAdjuster::_eye_callback(const my_hand_eye::Pose2DMightEndConstPtr &msg)
    {
        // 是否有not_change的变量，end为true时重置
        static bool rst = true;
        if (msg->end)
        {
            TwistMightEnd tme;
            tme.end = true;
            tme.velocity = geometry_msgs::Twist();
            cmd_vel_publisher_.publish(tme);
            if (!rst)
            {
                rst = true;
                changing_ = direction_theta;
                unchanging_ = direction_void;
                pid_ = PIDController({0}, {kp_eye_angular_},
                                     {ki_eye_angular_}, {kd_eye_angular_},
                                     {threshold_angular_}, {0.05}, {0.4});
                timer_.stop();
            }
            return;
        }
        else if (msg->pose.x == msg->not_change && msg->pose.y == msg->not_change &&
                 msg->pose.theta == msg->not_change)
        {
            TwistMightEnd tme;
            tme.end = false;
            tme.velocity = geometry_msgs::Twist();
            cmd_vel_publisher_.publish(tme);
            _add_pose_goal(*msg);
            return;
        }
        if (rst)
        {
            if (msg->pose.theta == msg->not_change)
            {
                unchanging_ = direction_theta;
                changing_ = direction_x;
                pid_ = PIDController({0, 0}, {kp_eye_linear_, kp_eye_angular_},
                                     {ki_eye_linear_, ki_eye_angular_},
                                     {kd_eye_linear_, kd_eye_angular_},
                                     {threshold_linear_x_, threshold_angular_}, {0.02, 0.05}, {0.2, 0.4});
            }
            else if (msg->pose.x == msg->not_change)
            {
                unchanging_ = direction_x;
            }
            else if (msg->pose.y == msg->not_change)
            {
                unchanging_ = direction_y;
            }
            rst = false;
        }
        if (!_add_pose_goal(*msg))
        {
            if (timer_.hasStarted())
                timer_.stop();
            return;
        }
        else if (!timer_.hasStarted())
            timer_.start();
        geometry_msgs::Pose2D pose;
        ros::Time stamp;
        if (!_get_pose_now(pose, stamp))
            return;
        std::vector<double> control;
        bool success;
        switch (changing_)
        {
        case direction_theta:
            if (pid_.update({pose.theta}, stamp, control, success))
            {
                if (debug_)
                    ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta
                                         << " changing:" << changing_ << " stamp:" << stamp.toSec() - ((int)stamp.toSec() / 10 * 10));
                TwistMightEnd tme;
                tme.end = false;
                tme.velocity.angular.z = control[0];
                cmd_vel_publisher_.publish(tme);
                if (success)
                {
                    if (unchanging_ == direction_x)
                    {
                        changing_ = direction_y;
                        pid_ = PIDController({0, 0, 0}, {kp_eye_linear_, kp_eye_linear_, kp_eye_angular_},
                                             {ki_eye_linear_, ki_eye_linear_, ki_eye_angular_},
                                             {kd_eye_linear_, kd_eye_linear_, kd_eye_angular_},
                                             {threshold_linear_x_, threshold_linear_y_, threshold_angular_}, {0.02, 0.02, 0.05}, {0.2, 0.2, 0.4});
                        return;
                    }
                    changing_ = direction_x;
                    pid_ = PIDController({0, 0}, {kp_eye_linear_, kp_eye_angular_},
                                         {ki_eye_linear_, ki_eye_angular_},
                                         {kd_eye_linear_, kd_eye_angular_},
                                         {threshold_linear_x_, threshold_angular_}, {0.02, 0.05}, {0.2, 0.4});
                }
            }
            break;

        case direction_x:
            if (pid_.update({pose.x, pose.theta}, stamp, control, success))
            {
                if (debug_)
                    ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta
                                         << " changing:" << changing_ << " stamp:" << stamp.toSec() - ((int)stamp.toSec() / 10 * 10));
                TwistMightEnd tme;
                tme.end = false;
                tme.velocity.linear.x = control[0];
                if (unchanging_ != direction_theta)
                    tme.velocity.angular.z = control[1];
                cmd_vel_publisher_.publish(tme);
                if (success && unchanging_ != direction_y)
                {
                    changing_ = direction_y;
                    pid_ = PIDController({0, 0, 0}, {kp_eye_linear_, kp_eye_linear_, kp_eye_angular_},
                                         {ki_eye_linear_, ki_eye_linear_, ki_eye_angular_},
                                         {kd_eye_linear_, kd_eye_linear_, kd_eye_angular_},
                                         {threshold_linear_x_, threshold_linear_y_, threshold_angular_}, {0.02, 0.02, 0.05}, {0.2, 0.2, 0.4});
                }
            }
            break;

        case direction_y:
            if (pid_.update({pose.x, pose.y, pose.theta}, stamp, control, success))
            {
                if (debug_)
                    ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta
                                         << " changing:" << changing_ << " stamp:" << stamp.toSec() - ((int)stamp.toSec() / 10 * 10));
                TwistMightEnd tme;
                tme.end = false;
                if (unchanging_ != direction_x)
                    tme.velocity.linear.x = control[0];
                tme.velocity.linear.y = control[1];
                if (unchanging_ != direction_theta)
                    tme.velocity.angular.z = control[2];
                cmd_vel_publisher_.publish(tme);
            }
            break;

        default:
            break;
        }
    }

    void VisionAdjuster::_timer_callback(const ros::TimerEvent &event)
    {
        geometry_msgs::Pose2D pose;
        ros::Time stamp;
        if (!_get_pose_now(pose, stamp))
            return;
        std::vector<double> control;
        bool success;
        switch (changing_)
        {
        case direction_theta:
            if (pid_.update({pose.theta}, stamp, control, success))
            {
                if (debug_)
                    ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta
                                         << " changing:" << changing_ << " stamp:" << stamp.toSec() - ((int)stamp.toSec() / 10 * 10));
                TwistMightEnd tme;
                tme.end = false;
                tme.velocity.angular.z = control[0];
                cmd_vel_publisher_.publish(tme);
            }
            break;

        case direction_x:
            if (pid_.update({pose.x, pose.theta}, stamp, control, success))
            {
                if (debug_)
                    ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta
                                         << " changing:" << changing_ << " stamp:" << stamp.toSec() - ((int)stamp.toSec() / 10 * 10));
                TwistMightEnd tme;
                tme.end = false;
                tme.velocity.linear.x = control[0];
                tme.velocity.angular.z = control[1];
                cmd_vel_publisher_.publish(tme);
            }
            break;

        case direction_y:
            if (pid_.update({pose.x, pose.y, pose.theta}, stamp, control, success))
            {
                if (debug_)
                    ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta
                                         << " changing:" << changing_ << " stamp:" << stamp.toSec() - ((int)stamp.toSec() / 10 * 10));
                TwistMightEnd tme;
                tme.end = false;
                tme.velocity.linear.x = control[0];
                tme.velocity.linear.y = control[1];
                tme.velocity.angular.z = control[2];
                cmd_vel_publisher_.publish(tme);
            }
            break;

        default:
            break;
        }
    }

    void VisionAdjuster::_dr_callback(params_PID_visionConfig &config, uint32_t level)
    {
        if (kp_eye_angular_ != config.kp_eye_angular)
            kp_eye_angular_ = config.kp_eye_angular;
        if (ki_eye_angular_ != config.ki_eye_angular)
            ki_eye_angular_ = config.ki_eye_angular;
        if (kd_eye_angular_ != config.kd_eye_angular)
            kd_eye_angular_ = config.kd_eye_angular;
        if (kp_eye_linear_ != config.kp_eye_linear)
            kp_eye_linear_ = config.kp_eye_linear;
        if (ki_eye_linear_ != config.ki_eye_linear)
            ki_eye_linear_ = config.ki_eye_linear;
        if (kd_eye_linear_ != config.kd_eye_linear)
            kd_eye_linear_ = config.kd_eye_linear;
    }

    bool VisionAdjuster::_add_pose_goal(const my_hand_eye::Pose2DMightEnd &pose)
    {
        geometry_msgs::PoseStamped pose_footprint;
        pose_footprint.header = pose.header;
        geometry_msgs::Pose p3D;
        // w = cos(theta/2) x = 0 y = 0 z = sin(theta/2)
        if (pose.pose.x == pose.not_change && pose.pose.y == pose.not_change &&
            pose.pose.theta == pose.not_change)
            pose_footprint.header.stamp = ros::Time();
        p3D.orientation.w = (pose.pose.theta == pose.not_change) ? 1 : cos(pose.pose.theta / 2);
        p3D.orientation.z = (pose.pose.theta == pose.not_change) ? 0 : sin(pose.pose.theta / 2);
        p3D.position.x = (pose.pose.x == pose.not_change) ? 0 : pose.pose.x;
        p3D.position.y = (pose.pose.y == pose.not_change) ? 0 : pose.pose.y;
        pose_footprint.pose = p3D;
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            buffer_.transform(pose_footprint, pose_goal_, target_frame_);
            pose_goal_.header.stamp = ros::Time();
        }
        catch (const std::exception &e)
        {
            ROS_WARN("_add_pose_goal exception:%s", e.what());
            return false;
        }
        return true;
    }

    bool VisionAdjuster::_get_pose_now(geometry_msgs::Pose2D &pose, ros::Time &stamp)
    {
        geometry_msgs::PoseStamped pose_footprint;
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            pose_footprint = buffer_.transform<geometry_msgs::PoseStamped>(pose_goal_, "base_footprint");
        }
        catch (const std::exception &e)
        {
            ROS_WARN("_get_pose_now exception:%s", e.what());
            return false;
        }
        pose.theta = unchanging_ != direction_theta ? atan2(pose_footprint.pose.orientation.z, pose_footprint.pose.orientation.w) * 2 : 0;
        pose.x = unchanging_ != direction_x ? pose_footprint.pose.position.x : 0;
        pose.y = unchanging_ != direction_y ? pose_footprint.pose.position.y : 0;
        stamp = pose_footprint.header.stamp;
        return true;
    }
} // namespace motion_controller
