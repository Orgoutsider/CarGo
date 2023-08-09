#include <motion_controller/TwistMightEnd.h>

#include "motion_controller/pid_controller.h"
#include "motion_controller/vision_adjuster.h"

namespace motion_controller
{
    VisionAdjuster::VisionAdjuster()
        : listener_(buffer_),
          not_change_(not_change_theta),
          kp_eye_angular_(1.2), ki_eye_angular_(0.0), kd_eye_angular_(1.2),
          kp_eye_linear_(0.5), ki_eye_linear_(0), kd_eye_linear_(0)
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        eye_subscriber_ = nh.subscribe<my_hand_eye::Pose2DMightEnd>(
            "/vision_eye", 3, &VisionAdjuster::_eye_callback, this);
        cmd_vel_publisher_ = nh.advertise<TwistMightEnd>("/cmd_vel_vision", 3);
        pnh.param<bool>("debug", debug_, false);
        if (debug_)
            dr_server_.setCallback(boost::bind(&VisionAdjuster::_dr_callback, this, _1, _2));
    }

    void VisionAdjuster::_eye_callback(const my_hand_eye::Pose2DMightEndConstPtr &msg)
    {
        // 是否有not_change的变量，end为true时重置
        static bool flag = false;
        static bool rst = true;
        static bool paning = false;
        static PIDController pid = PIDController({0}, {kp_eye_angular_},
                                                 {ki_eye_angular_}, {kd_eye_angular_},
                                                 {0.03}, {0.05}, {0.4});
        geometry_msgs::Pose2D pose = msg->pose;
        double change;
        if (msg->end)
        {
            TwistMightEnd tme;
            tme.end = true;
            tme.velocity = geometry_msgs::Twist();
            cmd_vel_publisher_.publish(tme);
            if (flag)
                flag = false;
            if (!rst)
            {
                rst = true;
                paning = false;
                pid = PIDController({0}, {kp_eye_angular_},
                                    {ki_eye_angular_}, {kd_eye_angular_},
                                    {0.03}, {0.05}, {0.4});
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
            return;
        }
        if (flag)
        {
            _get_change(change);
            switch (not_change_)
            {
            case not_change_theta:
                pose.theta = change;
                break;

            case not_change_x:
                pose.x = change;
                break;

            case not_change_y:
                pose.y = change;
                break;

            default:
                return;
            }
        }
        else if (rst)
        {
            if (pose.theta == msg->not_change)
            {
                if (!_get_transform())
                    return;
                not_change_ = not_change_theta;
                pose.theta = 0;
                flag = true;
                paning = true;
                pid = PIDController({0, 0, 0}, {kp_eye_linear_, kp_eye_linear_, kp_eye_angular_},
                                    {ki_eye_linear_, ki_eye_linear_, ki_eye_angular_},
                                    {kd_eye_linear_, kd_eye_linear_, kd_eye_angular_},
                                    {0.005, 0.005, 0.03}, {0.02, 0.02, 0.05}, {0.2, 0.2, 0.4});
            }
            else if (pose.x == msg->not_change)
            {
                if (!_get_transform())
                    return;
                not_change_ = not_change_x;
                pose.x = 0;
                flag = true;
            }
            else if (pose.y == msg->not_change)
            {
                if (!_get_transform())
                    return;
                not_change_ = not_change_y;
                pose.y = 0;
                flag = true;
            }
            rst = false;
        }
        std::vector<double> controll;
        bool success;
        if (paning)
        {
            if (pid.update({pose.x, pose.y, pose.theta}, msg->header.stamp,
                           controll, success))
            {
                ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta << " panning:" << paning);
                TwistMightEnd tme;
                tme.end = false;
                tme.velocity.linear.x = controll[0];
                tme.velocity.linear.y = controll[1];
                tme.velocity.angular.z = controll[2];
                cmd_vel_publisher_.publish(tme);
            }
        }
        else if (pid.update({pose.theta}, msg->header.stamp, controll, success))
        {
            ROS_INFO_STREAM("x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta << " panning:" << paning);
            TwistMightEnd tme;
            tme.end = false;
            tme.velocity.angular.z = controll[0];
            cmd_vel_publisher_.publish(tme);
            if (success)
            {
                paning = true;
                pid = PIDController({0, 0, 0}, {kp_eye_linear_, kp_eye_linear_, kp_eye_angular_},
                                    {ki_eye_linear_, ki_eye_linear_, ki_eye_angular_},
                                    {kd_eye_linear_, kd_eye_linear_, kd_eye_angular_},
                                    {0.005, 0.005, 0.03}, {0.02, 0.02, 0.05}, {0.2, 0.2, 0.4});
            }
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

    bool VisionAdjuster::_get_transform()
    {
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            tfs_ = buffer_.lookupTransform("odom_combined", "base_footprint", ros::Time(0));
        }
        catch (const std::exception &e)
        {
            ROS_WARN("get_transform exception:%s", e.what());
            return false;
        }
        return true;
    }

    bool VisionAdjuster::_get_change(double &change)
    {
        geometry_msgs::TransformStamped tfs_now;
        try
        {
            //   解析 base_footprint 中的点相对于 odom_combined 的坐标
            tfs_now = buffer_.lookupTransform("odom_combined", "base_footprint", ros::Time(0));
        }
        catch (const std::exception &e)
        {
            ROS_WARN("get_change exception:%s", e.what());
            return false;
        }
        switch (not_change_)
        {
        case not_change_x:
            change = tfs_.transform.translation.x - tfs_now.transform.translation.x;
            break;

        case not_change_y:
            change = tfs_.transform.translation.y - tfs_now.transform.translation.y;
            break;

        case not_change_theta:
            change = atan2(tfs_.transform.rotation.z, tfs_.transform.rotation.w) * 2 -
                     atan2(tfs_now.transform.rotation.z, tfs_now.transform.rotation.w) * 2;
            // 将change限定在(-pi,pi]之间
            change = (change <= -M_PI) ? change + M_PI : (change > M_PI ? change - M_PI : change);
            change = (change <= -M_PI) ? change + M_PI : (change > M_PI ? change - M_PI : change);
            break;

        default:
            ROS_ERROR("not_change_ invalid!");
            not_change_ = not_change_theta;
            change = 0;
            return false;
        }
        return true;
    }
} // namespace motion_controller
