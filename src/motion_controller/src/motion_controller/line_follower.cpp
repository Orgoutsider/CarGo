#include <std_msgs/Float64.h>
#include <motion_controller/TwistMightEnd.h>

#include "motion_controller/line_follower.h"

namespace motion_controller
{
    Point2d::Point2d(double x, double y) : x(x), y(y) {}

    LineFollower::LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : front_back_(false), front_left_(true), // 初始向左移动
          kp_(4.05), ki_(0), kd_(0.2),
          kp_adjust_(3.4), ki_adjust_(0), kd_adjust_(0.5),
          pid_({0}, {kp_}, {ki_}, {kd_}, {0.01}, {0.1}, {0.5}), bezier_ratio_(3),
          vel_max_(0.7), vel_(vel_max_), acc_(0.6), has_started(false), thresh_adjust_(0.015)
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
        if (kp_adjust_ != config.kp_adjust)
            kp_adjust_ = config.kp_adjust;
        if (ki_adjust_ != config.ki_adjust)
            ki_adjust_ = config.ki_adjust;
        if (kd_adjust_ != config.kd_adjust)
            kd_adjust_ = config.kd_adjust;
        if (thresh_adjust_ != config.thresh_adjust)
            thresh_adjust_ = config.thresh_adjust;
        if (bezier_ratio_ != config.bezier_ratio)
            bezier_ratio_ = config.bezier_ratio;
    }

    void LineFollower::publish_vel(double vel_1, double vel_2, double omega)
    {
        geometry_msgs::Twist twist;
        if (front_back_)
        {
            if (front_left_)
            {
                twist.linear.x = vel_1;
                twist.linear.y = vel_2;
            }
            else
            {
                twist.linear.x = -vel_1;
                twist.linear.y = -vel_2;
            }
        }
        else if (front_left_)
        {
            twist.linear.y = vel_1;
            twist.linear.x = -vel_2;
        }
        else
        {
            twist.linear.x = vel_2;
            twist.linear.y = -vel_1;
        }
        twist.angular.z = omega;
        TwistMightEnd tme;
        tme.velocity = twist;
        tme.end = false;
        cmd_vel_publisher_.publish(tme);
    }

    void LineFollower::limit_theta(double &theta)
    {
        // 将theta限制在target_theta_周围，防止不当的error
        theta = (theta > target_theta_ + M_PI)
                    ? theta - M_PI * 2
                    : (theta <= target_theta_ - M_PI ? theta + M_PI * 2 : theta);
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
                pid_ = PIDController({theta}, {theta_adjust ? kp_adjust_ : kp_},
                                     {theta_adjust ? ki_adjust_ : ki_},
                                     {theta_adjust ? kd_adjust_ : kd_},
                                     {theta_adjust ? thresh_adjust_ : 0.01}, {0.1},
                                     {theta_adjust ? 0.75 : 0.5});
                target_theta_ = theta;
                if (!theta_adjust)
                {
                    vel_ = std::min(vel_max_, sqrt(acc_ * dist));
                    length_ = (vel_ * vel_) / (2 * acc_);
                    dist_start_ = dist;
                }
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
            dist = dist > dist_start_ ? dist_start_ : dist;
            // 将theta限制在target_theta_周围，防止不当的error
            limit_theta(theta);
            bool flag = false;
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                flag = pid_.update({theta}, now, control, success);
            }
            if (flag)
            {
                double vel_n = vel_max_;
                if (dist_start_)
                {
                    if (dist > dist_start_ - length_)
                        vel_n = sqrt(2 * acc_ * (dist_start_ - dist)) * 0.6 + 0.4 * vel_max_;
                    else if (dist < 0.1)
                        vel_n = 0;
                    else if (dist < length_ + 0.1)
                        vel_n = std::max(sqrt(2 * acc_ * (dist - 0.1)), 0.1);
                }
                // ROS_INFO("%lf %lf %lf %lf", vel_n, dist, dist_start_, length_);
                // 需要增加一个负号来修正update的结果
                publish_vel(vel_n, 0, -control[0]);
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

    void LineFollower::veer(bool front_back, bool front_left)
    {
        boost::lock_guard<boost::recursive_mutex> lk(mtx);
        front_back_ = front_back;
        front_left_ = front_left;
    }

    Point2d LineFollower::point_find(Point2d Pb, Point2d P0, Point2d P1, double &bezier_t)
    {
        double k = (Pb.x - P0.x) / (P1.x - P0.x);
        bezier_t = sqrt(k);
        double y = (Pb.y + (pow(bezier_t, 2.0) - 2 * bezier_t) * P0.y) / pow((1 - bezier_t), 2.0);
        double x = P0.x;
        return Point2d(x, y);
    }

    double LineFollower::bezier_derivative(Point2d P0, Point2d P1, Point2d P2, double bezier_t)
    {
        double x = 2 * bezier_t * (P1.x - P0.x);
        double y = 2 * (1 - bezier_t) * (P2.y - P0.y);
        return y / x;
    }

    bool LineFollower::start_bezier(double theta, double dist, double dist_l)
    {
        if (dist_l < 0)
        {
            ROS_ERROR("Invalid dist: %lf", dist_l);
            return false;
        }
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
            pid_ = PIDController({theta}, {kp_}, {ki_}, {kd_}, {0.01}, {0.1}, {0.5});
            target_theta_ = theta;
            bezier_length_ = abs(bezier_ratio_ * dist_l);
            dist_start_ = dist;
        }
        geometry_msgs::Twist twist;
        double vel_1 = 0.1;
        double vel_2 = dist_l > 0 ? 0.1 : -0.1;
        publish_vel(vel_1, vel_2, 0);
        if (has_started)
        {
            ROS_WARN("Failed to start LineFollower");
            return false;
        }
        else
        {
            boost::lock_guard<boost::recursive_mutex> lk(mtx);
            has_started = true;
        }
        return true;
    }

    bool LineFollower::follow_bezier(double theta, double dist, double dist_l, const ros::Time &now)
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
            // 将theta限制在target_theta_周围，防止不当的error
            limit_theta(theta);
            bool flag = false;
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                flag = pid_.update({theta}, now, control, success);
            }
            if (flag)
            {
                if (dist > dist_start_)
                {
                    publish_vel(0.1, (dist_l > 0 ? 0.1 : -0.1), -control[0]);
                    return false;
                }
                double vel_1 = 0.1;
                Point2d p0(0, 0);
                Point2d p1(bezier_length_, 0);
                Point2d pb(dist_start_ - dist, dist_l);
                double t;
                Point2d p2 = point_find(pb, p0, p1, t);
                double vel_2 = bezier_derivative(p0, p1, p2, t) * vel_1;
                vel_2 = (vel_2 > 0.3) ? 0.3 : (vel_2 < -0.3 ? -0.3 : vel_2);
                if (abs(dist_l) < 0.03)
                {
                    vel_2 = 0;
                    boost::lock_guard<boost::recursive_mutex> lk(mtx);
                    vel_ = std::min(vel_max_, sqrt(acc_ * dist));
                    length_ = (vel_ * vel_) / (2 * acc_);
                    dist_start_ = dist;
                }
                else if (dist_start_ - dist > bezier_length_ || vel_2 == 0.3 || vel_2 == -0.3)
                {
                    vel_1 = 0;
                }
                ROS_INFO("%lf %lf %lf %lf %lf", vel_1, bezier_derivative(p0, p1, p2, t), vel_2, dist_l, bezier_length_);
                publish_vel(vel_1, vel_2, -control[0]);
            }
            return abs(dist_l) < 0.03;
        }
        else
            ROS_WARN_ONCE("Attempted to use 'follow_bezier' when follower has not started");
        return false;
    }

    bool LineFollower::start_then_adjust(double theta, double dist, const ros::Time &now)
    {
        bool success = false;
        std::vector<double> control;
        if (has_started)
        {
            ROS_INFO_STREAM("Adjusting ... theta: " << theta);
            static int cnt = 0;
            if ((dist < 0) && !debug)
            {
                ROS_ERROR("Invalid dist: %lf", dist);
                return true;
            }
            if (dist < 0.1)
            {
                return true;
            }
            // 将theta限制在target_theta_周围，防止不当的error
            limit_theta(theta);
            bool flag = false;
            {
                boost::lock_guard<boost::recursive_mutex> lk(mtx);
                flag = pid_.update({theta}, now, control, success);
            }
            if (flag)
            {
                double vel_1 = 0.1 * cos(target_theta_ - theta);
                double vel_2 = 0.1 * sin(target_theta_ - theta);
                if (success)
                {
                    if ((++cnt) >= 2)
                    {
                        publish_vel(0.1, 0, 0);
                        ROS_INFO("Adjust success!");
                        boost::lock_guard<boost::recursive_mutex> lk(mtx);
                        pid_ = PIDController({theta}, {kp_}, {ki_}, {kd_}, {0.01}, {0.1}, {0.5});
                        pid_.update({theta}, now, control, success);
                        // rst = true;
                        vel_ = std::min(vel_max_, sqrt(acc_ * dist));
                        length_ = (vel_ * vel_) / (2 * acc_);
                        dist_start_ = dist;
                        return true;
                    }
                    else
                        publish_vel(vel_1, vel_2, -control[0]);
                }
                else
                {
                    publish_vel(vel_1, vel_2, -control[0]);
                    if (cnt)
                        cnt = 0;
                }
            }
        }
        else
        {
            ROS_WARN_ONCE("Attempted to use 'start_then_adjust' when follower has not started");
        }
        return false;
    }
} // namespace motion_controller
