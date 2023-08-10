#include "motion_controller/pid_controller.h"

#define LIMIT(x, bound) x = (x < (bound)) ? ((x > -(bound)) ? x : -(bound)) : (bound)

namespace motion_controller
{
    
    PIDController::PIDController(std::vector<double> &&target,
                                 std::vector<double> &&p, std::vector<double> &&i, std::vector<double> &&d,
                                 std::vector<double> &&threshold, std::vector<double> &&integrator_max, std::vector<double> &&control_max)
        : target_(target), Kp_(p), Ki_(i), Kd_(d),
          threshold_(threshold), integrator_max_(integrator_max), control_max_(control_max)
    {
        ROS_ASSERT(target_.size() == Kp_.size());
        ROS_ASSERT(target_.size() == Ki_.size());
        ROS_ASSERT(target_.size() == Kd_.size());
        ROS_ASSERT(target_.size() == threshold_.size());
        ROS_ASSERT(target_.size() == integrator_max_.size());
        ROS_ASSERT(target_.size() == control_max_.size());
        last_error_.resize(target_.size(), 0);
        integrator_.resize(target_.size(), 0);
    }

    bool PIDController::update(std::vector<double> &&current,
                               const ros::Time &now, std::vector<double> &control, bool &success)
    {
        if (current.size() != target_.size())
        {
            ROS_ERROR("invalid size of current!");
            return false;
        }
        if (!control.empty())
            control.clear();
        if (last_time_.is_zero())
        {
            // PID第一次被调用，我们还不知道时间差
            // 没有控制信号被应用
            last_time_ = now;
            // 第一次的control不能用！
            control.resize(target_.size(), 0.0);
            return false;
        }
        control.reserve(target_.size());
        success = true;
        try
        {
            for (int i = 0; i < target_.size(); i++)
            {
                double error = current.at(i) - target_.at(i);
                // 偏差较小时停止移动
                if (abs(error) < threshold_.at(i))
                    error = 0;
                else if (success)
                    success = false;
                double p_out = Kp_.at(i) * error;
                double delta_t = (now - last_time_).toSec();
                // 误差的积分是 当前误差*时间差
                integrator_.at(i) = integrator_.at(i) + error * delta_t;
                LIMIT(integrator_.at(i), integrator_max_.at(i));
                double i_out = Ki_.at(i) * integrator_.at(i);
                // D是误差的差值 / 时间差
                double d_out = Kd_.at(i) * (error - last_error_.at(i)) / delta_t;
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
} // namespace motion_controller
