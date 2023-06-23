#include "motion_controller/line_follower.h"

namespace motion_controller
{
    LineFollower::LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : kp_(1.5), ki_(0.5), kd_(0.3),
          pid_({0}, {kp_}, {ki_}, {kd_}, {0.02}, {0.1}, {0.5}),
          linear_velocity_(0.2), has_started(false),
          motor_status(false)
    {
        pnh.param<bool>("param_modification", param_modification, false);
        cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_line", 3);
        if (param_modification)
        {
            dr_server_.setCallback(boost::bind(&LineFollower::_dr_callback, this, _1, _2));
            theta_publisher_ = nh.advertise<std_msgs::Float64>("theta", 5);
        }
    }

    void LineFollower::_dr_callback(lineConfig &config, uint32_t level)
    {
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
        if (motor_status != config.motor_status)
        {
            motor_status = config.motor_status;
            if (!motor_status && has_started)
                start(false);
        }
    }

    bool LineFollower::start(bool start, double theta)
    {
        if (start)
        {
            if (!has_started)
            {
                has_started = true;
                if (theta > M_PI * 3 / 4 || theta <= -M_PI * 3 / 4)
                    theta = M_PI;
                else if (theta > M_PI / 4)
                {
                    theta = M_PI / 2;
                }
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
            has_started = false;
            cmd_vel_publisher_.publish(geometry_msgs::Twist());
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
        std::vector<double> controll;
        if (has_started)
        {
            // 将theta限制在target_theta_周围，防止不当的error
            theta = (theta > target_theta_ + M_PI)
                        ? theta - M_PI * 2
                        : (theta <= target_theta_ - M_PI ? theta + M_PI * 2 : theta);
            if (pid_.update({theta}, now, controll, success))
            {
                geometry_msgs::Twist twist;
                twist.linear.x = linear_velocity_;
                // 需要增加一个负号来修正update的结果
                twist.angular.z = -controll[0];
                if (motor_status)
                    cmd_vel_publisher_.publish(twist);
            }
            if (param_modification)
            {
                std_msgs::Float64 msg;
                msg.data = theta;
                theta_publisher_.publish(msg);
            }
        }
    }
} // namespace motion_controller
