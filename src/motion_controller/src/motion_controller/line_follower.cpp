#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include "motion_controller/line_follower.h"

namespace motion_controller
{
    LineFollower::LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : front_back_(false), front_left_(true), // 初始向左移动
          kp_(3.8), ki_(0.4), kd_(0.3),
          pid_({0}, {kp_}, {ki_}, {kd_}, {0.02}, {0.1}, {0.5}),
          linear_velocity_(0.2), has_started(false),
          startup(false)
    {
        pnh.param<bool>("debug", debug, false);
        cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_line", 3);
        if (debug)
        {
            theta_publisher_ = nh.advertise<std_msgs::Float64>("theta", 5);
        }
    }

    void LineFollower::dr(routeConfig &config)
    {
        if (!debug)
            return;
        boost::lock_guard<boost::mutex> lk(mtx_);
        if (front_back_ != config.front_back)
            front_back_ = config.front_back;
        if (front_left_ != config.front_left)
            front_left_ = config.front_left;
        if (linear_velocity_ != config.linear_velocity)
        {
            linear_velocity_ = config.linear_velocity;
        }
        if (kp_ != config.kp)
            kp_ = config.kp;
        if (ki_ != config.ki)
            ki_ = config.ki;
        if (kd_ != config.kd)
            kd_ = config.kd;
    }

    bool LineFollower::start(bool start, double theta)
    {
        if (start)
        {
            if (!has_started)
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                has_started = true;
                if (theta > M_PI * 3 / 4 || theta <= -M_PI * 3 / 4)
                    theta = M_PI;
                else if (theta > M_PI / 4)
                    theta = M_PI / 2;
                else if (theta > -M_PI / 4)
                    theta = 0;
                else
                    theta = -M_PI / 2;

                pid_ = PIDController({theta}, {kp_}, {ki_}, {kd_}, {0.02}, {0.1}, {0.5});
                target_theta_ = theta;
            }
            else
            {
                ROS_WARN("Failed to start/stop LineFollower");
                return false;
            }
        }
        else if (has_started)
        {
            cmd_vel_publisher_.publish(geometry_msgs::Twist());
            boost::lock_guard<boost::mutex> lk(mtx_);
            has_started = false;
        }
        else
        {
            ROS_WARN("Failed to start/stop LineFollower");
            return false;
        }
        return true;
    }

    void LineFollower::follow(double theta, const ros::Time &now)
    {
        bool success;
        std::vector<double> control;
        if (has_started)
        {
            // 将theta限制在target_theta_周围，防止不当的error
            theta = (theta > target_theta_ + M_PI)
                        ? theta - M_PI * 2
                        : (theta <= target_theta_ - M_PI ? theta + M_PI * 2 : theta);
            bool flag = false;
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                flag = pid_.update({theta}, now, control, success);
            }
            if (flag)
            {
                geometry_msgs::Twist twist;
                if (front_back_)
                {
                    if (front_left_)
                        twist.linear.x = linear_velocity_;
                    else
                        twist.linear.x = -linear_velocity_;
                }
                else if (front_left_)
                    twist.linear.y = linear_velocity_;
                else
                    twist.linear.y = -linear_velocity_;
                // 需要增加一个负号来修正update的结果
                twist.angular.z = -control[0];
                if (startup)
                    cmd_vel_publisher_.publish(twist);
            }
            if (debug)
            {
                std_msgs::Float64 msg;
                msg.data = theta;
                theta_publisher_.publish(msg);
            }
        }
        else
            ROS_WARN_ONCE("Attempted to use 'follow' when follower has not started");
    }

    void LineFollower::veer(bool front_back, bool front_left)
    {
        boost::lock_guard<boost::mutex> lk(mtx_);
        front_back_ = front_back;
        front_left_ = front_left;
    }
} // namespace motion_controller
