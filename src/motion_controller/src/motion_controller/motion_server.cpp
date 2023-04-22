#include "motion_controller/motion_server.h"

namespace motion_controller
{
    MotionServer::MotionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : server_(nh, "Move", boost::bind(&MotionServer::_execute_callback, this, _1), false),
          listener_(buffer_)
    {
        server_.registerPreemptCallback(boost::bind(&MotionServer::_preempt_callback, this));
        server_.start();
        cmd_vel_publisher_ = nh.advertise<TwistMightEnd>("/cmd_vel_srv", 3);
    }

    void MotionServer::_execute_callback(const MoveGoalConstPtr &goal)
    {
        ros::Rate rate(10);
        MoveFeedback feedback;
        TwistMightEnd tme;
        // 计算目标位姿
        while (!_add_pose_goal(goal->pose) && ros::ok())
        {
            feedback.is_paning = false;
            feedback.pose_now = goal->pose;
            server_.publishFeedback(feedback);
            // 组织发布速度消息
            tme.end = false;
            cmd_vel_publisher_.publish(tme);
            rate.sleep();
        }
        // 转弯
        geometry_msgs::Pose2D pose = goal->pose;
        ros::Time now = ros::Time::now();
        PIDController pid1({0}, {kp_angular_}, {ki_angular_}, {kd_angular_}, {0.1}, {0.2}, {0.6});
        bool success = false;
        std::vector<double> controll;
        while (!success && ros::ok())
        {
            if (_get_pose_now(pose, now))
            {
                // pose.theta从-pi到+pi
                if (pid1.update({pose.theta}, now, controll, success)) // pid更新成功
                {
                    // 组织发布速度消息
                    geometry_msgs::Twist twist;
                    twist.angular.z = controll[0];
                    tme.velocity = twist;
                }
                else
                {
                    // 组织发布速度消息
                    tme.velocity = geometry_msgs::Twist();
                }
            }
            else
            {
                tme.velocity = geometry_msgs::Twist();
            }
            tme.end = false;
            cmd_vel_publisher_.publish(tme);
            feedback.is_paning = false;
            feedback.pose_now = pose;
            server_.publishFeedback(feedback);
            rate.sleep();
        }
        // 平移
        PIDController pid2({0, 0, 0}, {kp_linear_, kp_linear_, kp_angular_},
                           {ki_linear_, ki_linear_, ki_angular_}, {kd_linear_, kd_linear_, kd_angular_},
                           {0.05, 0.05, 0.1}, {0.1, 0.1, 0.2}, {0.3, 0.3, 0.6});
        success = false;
        while (!success && ros::ok())
        {
            if (_get_pose_now(pose, now))
            {
                if (pid2.update({pose.x, pose.y, pose.theta}, now, controll, success))
                {
                    // 组织发布速度消息
                    geometry_msgs::Twist twist;
                    twist.linear.x = controll[0];
                    twist.linear.y = controll[1];
                    twist.angular.z = controll[2];
                    tme.velocity = twist;
                }
                else
                {
                    // 组织发布速度消息
                    tme.velocity = geometry_msgs::Twist();
                }
            }
            else
            {
                // 组织发布速度消息
                tme.velocity = geometry_msgs::Twist();
            }
            tme.end = false;
            cmd_vel_publisher_.publish(tme);
            feedback.is_paning = true;
            feedback.pose_now = pose;
            server_.publishFeedback(feedback);
            rate.sleep();
        }
        tme.end = true;
        tme.velocity = geometry_msgs::Twist();
        cmd_vel_publisher_.publish(tme);
        _get_pose_now(pose, now);
        MoveResult result;
        result.pose_final = pose;
        server_.setSucceeded(result, "Move success!");
        ROS_INFO("Move success!");
    }

    void MotionServer::_preempt_callback()
    {
        ROS_WARN("Preempt Requested!");
        TwistMightEnd tme;
        tme.end = true;
        tme.velocity = geometry_msgs::Twist();
        cmd_vel_publisher_.publish(tme);
        server_.setPreempted(MoveResult(), "Got preempted by a new goal");
    }

    bool MotionServer::_add_pose_goal(geometry_msgs::Pose2D pose)
    {
        geometry_msgs::PoseStamped pose_footprint;
        pose_footprint.header.frame_id = "base_footprint";
        pose_footprint.header.stamp = ros::Time();
        // 三维位姿
        geometry_msgs::Pose p3D;
        // w = cos(theta/2) x = 0 y = 0 z = sin(theta/2)
        p3D.orientation.w = cos(pose.theta / 2);
        p3D.orientation.z = cos(pose.theta / 2);
        // 由于延时可能转换失败报错
        try
        {
            pose_goal_ = buffer_.transform(pose_footprint, "odom_combined");
            pose_goal_.header.stamp = ros::Time();
        }
        catch (const std::exception &e)
        {
            ROS_INFO("Transform error:%s", e.what());
            return false;
        }
        return true;
    }

    bool MotionServer::_get_pose_now(geometry_msgs::Pose2D &pose, ros::Time &now)
    {
        geometry_msgs::PoseStamped pose_footprint;
        // 由于延时可能转换失败报错
        try
        {
            pose_footprint = buffer_.transform<geometry_msgs::PoseStamped>(pose_goal_, "base_footprint");
        }
        catch (const std::exception &e)
        {
            ROS_INFO("Transform error:%s", e.what());
            return false;
        }
        // w = cos(theta/2) x = 0 y = 0 z = sin(theta/2)
        pose.theta = atan2(pose_footprint.pose.orientation.z, pose_footprint.pose.orientation.w);
        pose.x = pose_footprint.pose.position.x;
        pose.y = pose_footprint.pose.position.y;
        now = pose_footprint.header.stamp;
        return true;
    }
} // namespace motion_controller
