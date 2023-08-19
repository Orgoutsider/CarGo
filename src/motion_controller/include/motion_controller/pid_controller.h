#ifndef _PID_CONTROLLER_
#define _PID_CONTROLLER_

#include <ros/ros.h>

namespace motion_controller
{
    typedef std::vector<double> Vec;

    class PIDController
    {
    protected:
        Vec target_;
        Vec Kp_;
        Vec Ki_;
        Vec Kd_;
        Vec last_error_;
        Vec thresh_;
        Vec int_;
        Vec int_max_;
        Vec control_max_;
        ros::Time last_time_;

    public:
        PIDController(Vec &&target, Vec &&p, Vec &&i, Vec &&d,
                      Vec &&thresh, Vec &&int_max, Vec &&control_max);
        // 注意调用update必须等到返回值为true时！
        bool update(Vec &&current, const ros::Time &now,
                    Vec &control, bool &success);
    };

    class PIDControllerWithFilter : public PIDController
    {
    private:
        Vec *int_d_;
        Vec limiting_freq_;

    public:
        PIDControllerWithFilter(Vec &&target,  Vec &&p, Vec &&i, Vec &&d,
                                Vec &&thresh, Vec &&int_max,
                                Vec &&control_max, Vec &&limiting_freq);
        // 注意调用update必须等到返回值为true时！
        bool update(Vec &&current, const ros::Time &now,
                    Vec &control, bool &success);
    };
} // namespace motion_controller

#endif // !_PID_CONTROLLER_