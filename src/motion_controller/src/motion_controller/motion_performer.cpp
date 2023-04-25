#include "motion_controller/motion_performer.h"

namespace motion_controller
{
    MotionPerformer::MotionPerformer() : level_(level_line)
    {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 3);
        subscriber_line_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_line", 1, &MotionPerformer::_line_callback, this);
        subscriber_vision_ = nh.subscribe<TwistMightEnd>("/cmd_vel_vision", 1, &MotionPerformer::_vision_callback, this);
        subscriber_service_ = nh.subscribe<TwistMightEnd>("/cmd_vel_srv", 1, &MotionPerformer::_service_callback, this);
    }

    void MotionPerformer::_line_callback(const geometry_msgs::TwistConstPtr &msg)
    {
        if (level_line >= level_)
            publisher_.publish<geometry_msgs::Twist>(*msg);
        else
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }

    void MotionPerformer::_vision_callback(const TwistMightEndConstPtr &msg)
    {
        if (level_vision >= level_)
        {
            if (level_vision > level_)
            {
                // std::lock_guard对象构造时，自动调用mtx.lock()进行上锁
                // std::lock_guard对象析构时，自动调用mtx.unlock()释放锁
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_vision;
            }
            else if (msg->end)
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_line;
            }
            publisher_.publish<geometry_msgs::Twist>(msg->velocity);
        }
        else
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }

    void MotionPerformer::_service_callback(const TwistMightEndConstPtr &msg)
    {
        if (level_service >= level_)
        {
            if (level_service > level_)
            {
                // std::lock_guard对象构造时，自动调用mtx.lock()进行上锁
                // std::lock_guard对象析构时，自动调用mtx.unlock()释放锁
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_service;
            }
            else if (msg->end)
            {
                boost::lock_guard<boost::mutex> lk(mtx_);
                level_ = level_line;
            }
            publisher_.publish<geometry_msgs::Twist>(msg->velocity);
        }
    }
} // namespace motion_controller
