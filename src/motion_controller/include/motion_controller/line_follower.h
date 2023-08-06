#ifndef _LINE_FOLLOWER_H_
#define _LINE_FOLLOWER_H_

#include "motion_controller/field_guide.h"
#include "motion_controller/pid_controller.h"

namespace motion_controller
{
    class LineFollower
    {
    private:
        bool front_back_;                  // 是否前后向移动
        bool front_left_;                  // 是否左向或前向移动
        double kp_;                        // pid参数p
        double ki_;                        // pid参数i
        double kd_;                        // pid参数d
        double target_theta_;              // 目标角度，开启时设定
        double linear_velocity_;           // 线速度
        ros::Publisher cmd_vel_publisher_; // 底盘速度话题发布
        ros::Publisher theta_publisher_;   // 调试时使用，观察theta变化
        PIDController pid_;

    public:
        LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        bool debug;       // 动态调参，与子类（MotionController）共用
        bool startup;     // 调参，即停选项
        bool has_started; // 是否已经启动
        // 用于走直线动态调参
        void dr(routeConfig &config);
        // 启动并输入theta，自动转成目标角度
        bool start(bool start, double theta = 0);
        // 使用pid走直线，如果LineFollower尚未启动，则不做处理
        void follow(double theta, const ros::Time &now);
        void veer(bool front_back, bool front_left);
    };

} // namespace motion_controller

#endif // !_LINE_FOLLOWER_H_