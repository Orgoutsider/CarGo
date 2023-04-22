#ifndef _PID_CONTROLLER_
#define _PID_CONTROLLER_

#include <ros/ros.h>

namespace motion_controller
{
    class PIDController
    {
    private:
        std::vector<double> target_;
        std::vector<double> Kp_;
        std::vector<double> Ki_;
        std::vector<double> Kd_;
        std::vector<double> threshold_;
        std::vector<double> last_error_;
        std::vector<double> integrator_;
        std::vector<double> integrator_max_;
        std::vector<double> controll_max_;
        ros::Time last_time_;

    public:
        PIDController(std::vector<double> &&target,
                      std::vector<double> &&p, std::vector<double> &&i, std::vector<double> &&d,
                      std::vector<double> &&threshold, std::vector<double> &&integrator_max, std::vector<double> &&controll_max);
        bool update(std::vector<double> &&current, ros::Time &now,
                    std::vector<double> &controll, bool &success);
    };

} // namespace motion_controller

#endif // !_PID_CONTROLLER_