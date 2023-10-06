#include <std_msgs/Float64.h>
#include <motion_controller/TwistMightEnd.h>

#include "motion_controller/line_follower.h"

namespace motion_controller
{
    LineFollower::LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : front_back_(false), front_left_(true), // 初始向左移动
          kp_(4.05), ki_(0), kd_(0.2),
          pid_({0}, {kp_}, {ki_}, {kd_}, {0.01}, {0.1}, {0.5}),
          vel_max_(0.5), vel_(vel_max_), acc_(0.6), has_started(false), thresh_adjust_(0.05)
    {
        pnh.param<bool>("debug", debug, false);
        cmd_vel_publisher_ = nh.advertise<TwistMightEnd>("/cmd_vel_line", 3);
        if (debug)
        {
            theta_publisher_ = nh.advertise<std_msgs::Float64>("theta", 5);
        }
    }

    void LineFollower::dr(routeConfig &config)
    {
        if (!debug)
            return;
        boost::lock_guard<boost::recursive_mutex> lk(mtx);
        if (front_back_ != config.front_back)
            front_back_ = config.front_back;
        if (front_left_ != config.front_left)
            front_left_ = config.front_left;
        if (vel_max_ != config.vel_max)
        {
            vel_max_ = config.vel_max;
        }
        if (acc_ != config.acc)
            acc_ = config.acc;
        if (kp_ != config.kp)
            kp_ = config.kp;
        if (ki_ != config.ki)
            ki_ = config.ki;
        if (kd_ != config.kd)
            kd_ = config.kd;
        if (thresh_adjust_ != config.thresh_adjust)
            thresh_adjust_ = config.thresh_adjust;
    }

    bool LineFollower::start(bool start, double theta, double dist, double theta_adjust)
    {
        if (start)
        {
            if (dist < 0)
            {
                ROS_ERROR("Invalid dist: %lf", dist);
                return false;
            }
            theta = theta + theta_adjust;
            if (theta > M_PI * 3 / 4 || theta <= -M_PI * 3 / 4)
                theta = M_PI;
            else if (theta > M_PI / 4)
                theta = M_PI / 2;
            else if (theta > -M_PI / 4)
                theta = 0;
            else
                theta = -M_PI / 2;
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                pid_ = PIDController({theta}, {kp_}, {ki_}, {kd_},
                                     {theta_adjust ? thresh_adjust_ : 0.01}, {0.1}, {0.5});
                target_theta_ = theta;
                vel_ = std::min(vel_max_, sqrt(acc_ * dist));
                length_ = (vel_ * vel_) / (2 * acc_);
                dist_start = dist;
            }
            if (has_started)
            {
                ROS_WARN("Failed to start/stop LineFollower");
                return false;
            }
            else
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                has_started = true;
            }
        }
        else if (has_started) // stop
        {
            TwistMightEnd tme;
            tme.end = true;
            cmd_vel_publisher_.publish(tme);
            boost::lock_guard<boost::recursive_mutex> lk(mtx);
            has_started = false;
        }
        else
        {
            ROS_WARN("Failed to start/stop LineFollower");
            return false;
        }
        return true;
    }

    bool LineFollower::follow(double theta, double dist, const ros::Time &now)
    {
        bool success;
        std::vector<double> control;
        if (has_started)
        {
            if ((dist < 0) && !debug)
            {
                ROS_ERROR("Invalid dist: %lf", dist);
                return false;
            }
            dist = dist > dist_start ? dist_start : dist;
            // 将theta限制在target_theta_周围，防止不当的error
            theta = (theta > target_theta_ + M_PI)
                        ? theta - M_PI * 2
                        : (theta <= target_theta_ - M_PI ? theta + M_PI * 2 : theta);
            bool flag = false;
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                flag = pid_.update({theta}, now, control, success);
            }
            if (flag)
            {
                double vel_n = vel_max_;
                if (dist_start)
                {
                    if (dist > dist_start - length_)
                        vel_n = sqrt(2 * acc_ * (dist_start - dist)) * 0.8 + 0.2;
                    else if (dist < 0.1)
                        vel_n = 0;
                    else if (dist < length_)
                        vel_n = std::max(sqrt(2 * acc_ * dist) * 1.2 - 0.2, 0.2);
                }
                // ROS_INFO("%lf %lf %lf %lf", vel_n, dist, dist_start, length_);
                geometry_msgs::Twist twist;
                if (front_back_)
                {
                    if (front_left_)
                        twist.linear.x = vel_n;
                    else
                        twist.linear.x = -vel_n;
                }
                else if (front_left_)
                    twist.linear.y = vel_n;
                else
                    twist.linear.y = -vel_n;
                // 需要增加一个负号来修正update的结果
                twist.angular.z = -control[0];
                TwistMightEnd tme;
                tme.velocity = twist;
                tme.end = false;
                cmd_vel_publisher_.publish(tme);
            }
            if (debug)
            {
                std_msgs::Float64 msg;
                msg.data = theta;
                theta_publisher_.publish(msg);
            }
            return dist < 0.1;
        }
        else
            ROS_WARN_ONCE("Attempted to use 'follow' when follower has not started");
        return false;
    }

    bool LineFollower::stop_and_adjust(double theta, const ros::Time &now)
    {
        bool success = false;
        std::vector<double> control;
        static bool rst = true;
        if (has_started)
        {
            ROS_INFO_STREAM("Adjusting ... theta: " << theta);
            static ros::Time time;
            if (rst)
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                time = now;
                rst = false;
            }
            if ((now - time).toSec() >= 5)
            {
                ROS_WARN("Adjust timeout!");
                // start(false);
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                pid_.change_thresh({0.01});
                rst = true;
                return true;
            }
            // 将theta限制在target_theta_周围，防止不当的error
            theta = (theta > target_theta_ + M_PI)
                        ? theta - M_PI * 2
                        : (theta <= target_theta_ - M_PI ? theta + M_PI * 2 : theta);
            bool flag = false;
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                flag = pid_.update({theta}, now, control, success);
            }
            if (flag)
            {
                double vel_n = 0.1;
                geometry_msgs::Twist twist;
                if (front_back_)
                {
                    if (front_left_)
                        twist.linear.x = vel_n;
                    else
                        twist.linear.x = -vel_n;
                }
                else if (front_left_)
                    twist.linear.y = vel_n;
                else
                    twist.linear.y = -vel_n;
                // 需要增加一个负号来修正update的结果
                twist.angular.z = -control[0];
                TwistMightEnd tme;
                tme.velocity = twist;
                tme.end = false;
                if (success)
                {
                    ROS_INFO("Adjust success!");
                    boost::lock_guard<boost::recursive_mutex> lk(mtx);
                    pid_.change_thresh({0.01});
                    rst = true;
                    // start(false);
                }
                cmd_vel_publisher_.publish(tme);
            }
        }
        else
        {
            ROS_WARN_ONCE("Attempted to use 'stop_and_adjust' when follower has not started");
            return false;
        }
        return success;
    }

    void LineFollower::veer(bool front_back, bool front_left)
    {
        boost::lock_guard<boost::recursive_mutex> lk(mtx);
        front_back_ = front_back;
        front_left_ = front_left;
    }
} // namespace motion_controller
