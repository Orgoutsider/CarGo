#include "motion_controller/pid_controller.h"

#define LIMIT(x, bound) x = (x < (bound)) ? ((x > -(bound)) ? x : -(bound)) : (bound)

namespace motion_controller
{

    PIDController::PIDController(Vec &&target, Vec &&p, Vec &&i, Vec &&d,
                                 Vec &&thresh, Vec &&int_max, Vec &&control_max)
        : target_(target), Kp_(p), Ki_(i), Kd_(d),
          thresh_(thresh), int_max_(int_max), control_max_(control_max)
    {
        ROS_ASSERT(target_.size() == Kp_.size());
        ROS_ASSERT(target_.size() == Ki_.size());
        ROS_ASSERT(target_.size() == Kd_.size());
        ROS_ASSERT(target_.size() == thresh_.size());
        ROS_ASSERT(target_.size() == int_max_.size());
        ROS_ASSERT(target_.size() == control_max_.size());
        last_error_.resize(target_.size(), 0);
        int_.resize(target_.size(), 0);
    }

    bool PIDController::update(Vec &&current,
                               const ros::Time &now, Vec &control, bool &success)
    {
        if (current.size() != target_.size())
        {
            ROS_WARN("invalid size of current! current size:%ld, target size:%ld", current.size(), target_.size());
            return false;
        }
        if (!control.empty())
            control.clear();
        success = true;
        if (last_time_.is_zero())
        {
            // PID第一次被调用，我们还不知道时间差
            // 没有控制信号被应用
            last_time_ = now;
            // 第一次的control不能用！
            control.resize(target_.size(), 0.0);
            try
            {
                for (int i = 0; i < target_.size(); i++)
                {
                    if (abs(current.at(i) - target_.at(i)) >= thresh_.at(i) && success)
                        success = false;
                }
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("%s", e.what());
            }
            return false;
        }
        control.reserve(target_.size());
        try
        {
            for (int i = 0; i < target_.size(); i++)
            {
                double error = current.at(i) - target_.at(i);
                // 偏差较小时停止移动
                if (abs(error) < thresh_.at(i))
                    error = 0;
                else if (success)
                    success = false;
                double p_out = Kp_.at(i) * error;
                double delta_t = (now - last_time_).toSec();
                // 误差的积分是 当前误差*时间差
                int_.at(i) = int_.at(i) + error * delta_t;
                LIMIT(int_.at(i), int_max_.at(i));
                double i_out = Ki_.at(i) * int_.at(i);
                // D是误差的差值 / 时间差
                double d_out = delta_t == 0 ? 0 : Kd_.at(i) * (error - last_error_.at(i)) / delta_t;
                last_error_.at(i) = error;
                double out = p_out + i_out + d_out;
                LIMIT(out, control_max_.at(i));
                control.push_back(out);
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
        last_time_ = now;
        return true;
    }

    void PIDController::change_thresh(Vec &&thresh)
    {
        ROS_ASSERT(thresh.size() == thresh_.size());
        thresh_ = thresh;
    }

    PIDControllerWithFilter::PIDControllerWithFilter(Vec &&target, Vec &&p, Vec &&i, Vec &&d,
                                                     Vec &&thresh, Vec &&int_max,
                                                     Vec &&control_max, Vec &&limiting_freq)
        : PIDController(std::forward<Vec>(target),
                        std::forward<Vec>(p), std::forward<Vec>(i), std::forward<Vec>(d),
                        std::forward<Vec>(thresh), std::forward<Vec>(int_max),
                        std::forward<Vec>(control_max)),
          limiting_freq_(limiting_freq)
    {
        ROS_ASSERT(target_.size() == limiting_freq_.size());
    }

    bool PIDControllerWithFilter::update(Vec &&current, const ros::Time &now,
                                         Vec &control, bool &success)
    {
        if (current.size() != target_.size())
        {
            ROS_WARN("invalid size of current! current size:%ld, target size:%ld", current.size(), target_.size());
            return false;
        }
        if (!control.empty())
            control.clear();
        success = true;
        if (last_time_.is_zero())
        {
            // PID第一次被调用，我们还不知道时间差
            // 没有控制信号被应用
            last_time_ = now;
            // 第一次的control不能用！
            control.resize(target_.size(), 0.0);
            try
            {
                for (int i = 0; i < target_.size(); i++)
                {
                    if (abs(current.at(i) - target_.at(i)) >= thresh_.at(i) && success)
                        success = false;
                }
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("%s", e.what());
            }
            return false;
        }
        control.reserve(target_.size());
        try
        {
            for (int i = 0; i < target_.size(); i++)
            {
                double error = current.at(i) - target_.at(i);
                // 偏差较小时停止移动
                if (abs(error) < thresh_.at(i))
                    error = 0;
                else if (success)
                    success = false;
                double p_out = Kp_.at(i) * error;
                double delta_t = (now - last_time_).toSec();
                // 误差的积分是 当前误差*时间差
                int_.at(i) += error * delta_t;
                LIMIT(int_.at(i), int_max_.at(i));
                double i_out = Ki_.at(i) * int_.at(i);
                double derr = limiting_freq_.at(i) * (error - last_error_.at(i));
                double d_out = Kd_.at(i) * derr;
                last_error_.at(i) += derr * delta_t;
                double out = p_out + i_out + d_out;
                LIMIT(out, control_max_.at(i));
                control.push_back(out);
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
        last_time_ = now;
        return true;
    }
} // namespace motion_controller
