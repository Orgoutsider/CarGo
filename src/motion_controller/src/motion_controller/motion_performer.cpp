#include "motion_controller/motion_performer.h"

namespace motion_controller
{
    MotionPerformer::MotionPerformer() : level_(level_line), timeout_(1.0)
    {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        subscriber_line_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_line", 3, &MotionPerformer::_line_callback, this);
        subscriber_vision_ = nh.subscribe<TwistMightEnd>("/cmd_vel_vision", 3, &MotionPerformer::_vision_callback, this);
        subscriber_service_ = nh.subscribe<TwistMightEnd>("/cmd_vel_srv", 3, &MotionPerformer::_service_callback, this);
    }

    void MotionPerformer::_line_callback(const geometry_msgs::TwistConstPtr &msg)
    {
        if (level_line >= level_)
            publisher_.publish<geometry_msgs::Twist>(*msg);
        else if ((level_ == level_vision && ((ros::Time::now() - time_vision_).toSec() < timeout_ ||
                                             time_vision_.is_zero())) ||
                 (level_ == level_service && ((ros::Time::now() - time_service_).toSec() < timeout_ ||
                                              time_service_.is_zero())))
        {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        }
        else
        {
            ROS_WARN("Timeout! Level update.");
            boost::lock_guard<boost::mutex> lk(mtx_);
            level_ = level_line;
        }
    }

    void MotionPerformer::_vision_callback(const TwistMightEndConstPtr &msg)
    {
        if (level_vision >= level_)
        {
            if (level_vision > level_)
            {
                // boost::lock_guard对象构造时，自动调用mtx.lock()进行上锁
                // boost::lock_guard对象析构时，自动调用mtx.unlock()释放锁
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_vision;
            }
            else if (msg->end)
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_line;
            }
            publisher_.publish<geometry_msgs::Twist>(msg->velocity);
            boost::lock_guard<boost::mutex> lk(mtx_);
            time_vision_ = ros::Time::now();
        }
        else if ((ros::Time::now() - time_service_).toSec() < timeout_ || time_service_.is_zero())
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        else
        {
            ROS_WARN("Timeout! Level update.");
            boost::lock_guard<boost::mutex> lk(mtx_);
            level_ = level_vision;
        }
    }

    void MotionPerformer::_service_callback(const TwistMightEndConstPtr &msg)
    {
        if (level_service >= level_)
        {
            if (level_service > level_)
            {
                // boost::lock_guard对象构造时，自动调用mtx.lock()进行上锁
                // boost::lock_guard对象析构时，自动调用mtx.unlock()释放锁
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_service;
            }
            else if (msg->end)
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_line;
            }
            publisher_.publish<geometry_msgs::Twist>(msg->velocity);
            boost::lock_guard<boost::mutex> lk(mtx_);
            time_service_ = ros::Time::now();
        }
    }
} // namespace motion_controller
