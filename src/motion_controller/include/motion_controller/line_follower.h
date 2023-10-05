#ifndef _LINE_FOLLOWER_H_
#define _LINE_FOLLOWER_H_

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

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
        double vel_max_;                   // 线速度阈值
        double vel_;                       // 此轮运动线速度最大值
        double acc_;                       // 线加速度
        double length_;                    // 加速减速过程中位移
        double dist_start;                 // 起始距离
        double thresh_adjust_;             // 调整过程中的角度阈值
        ros::Publisher cmd_vel_publisher_; // 底盘速度话题发布
        ros::Publisher theta_publisher_;   // 调试时使用，观察theta变化
        PIDController pid_;

    public:
        LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        boost::recursive_mutex mtx; // 递归锁可以允许一个线程对同一互斥量多次加锁
        bool debug;                 // 动态调参，与子类（MotionController）共用
        bool has_started;           // 是否已经启动
        // 用于走直线动态调参
        void dr(routeConfig &config);
        // 启动并输入theta，自动转成目标角度
        bool start(bool start, double theta = 0, double dist = 0, double theta_adjust = 0);
        // 使用pid走直线，如果LineFollower尚未启动，则不做处理，如果到达输出ture(dist == 0不停止)
        bool follow(double theta, double dist, const ros::Time &now);
        // 停止并调整姿态
        bool stop_and_adjust(double theta, const ros::Time &now);
        // 改变行驶方向
        void veer(bool front_back, bool front_left);
    };

} // namespace motion_controller

#endif // !_LINE_FOLLOWER_H_