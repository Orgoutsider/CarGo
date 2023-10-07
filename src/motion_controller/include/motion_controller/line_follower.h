#ifndef _LINE_FOLLOWER_H_
#define _LINE_FOLLOWER_H_

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

#include "motion_controller/field_guide.h"
#include "motion_controller/pid_controller.h"

namespace motion_controller
{
    struct Point2d
    {
        double x, y;
        Point2d(double x, double y);
    };

    class LineFollower
    {
    private:
        bool front_back_;                  // 是否前后向移动
        bool front_left_;                  // 是否左向或前向移动
        double kp_;                        // pid参数p
        double ki_;                        // pid参数i
        double kd_;                        // pid参数d
        double kp_adjust_;                 // pid参数p（调整时）
        double ki_adjust_;                 // pid参数i（调整时）
        double kd_adjust_;                 // pid参数d（调整时）
        double target_theta_;              // 目标角度，开启时设定
        double vel_max_;                   // 线速度阈值
        double vel_;                       // 此轮运动线速度最大值
        double acc_;                       // 线加速度
        double length_;                    // 加速减速过程中位移
        double dist_start_;                // 起始距离
        double thresh_adjust_;             // 调整过程中的角度阈值
        double bezier_ratio_;              // 起始点到目标点的斜率
        double bezier_length_;             // 贝塞尔曲线的目标点
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
        void publish_vel(double vel_1, double vel_2, double omega);
        void limit_theta(double &theta);
        // 启动并输入theta，自动转成目标角度
        bool start(bool start, double theta = 0, double dist = 0, double theta_adjust = 0);
        // 使用pid走直线，如果LineFollower尚未启动，则不做处理，如果到达输出ture(dist == 0不停止)
        bool follow(double theta, double dist, const ros::Time &now);
        // 边走边转弯
        bool start_then_adjust(double theta, double dist, const ros::Time &now);
        // 改变行驶方向
        void veer(bool front_back, bool front_left);
        // 给定点寻找控制点，给定点为Pb，P0-P1为已知控制点，P2为未知控制点，bezier_t为通过该函数计算得到的Pb点对应的参数
        // 注意：控制点顺序为P2-P0-P1，改变控制点则寻找控制点函数也需要改变
        Point2d point_find(Point2d Pb, Point2d P0, Point2d P1, double &bezier_t);
        // 求出给定贝塞尔参数对应点处的导数，P0-P2为控制点
        // 注意：控制点顺序为P2-P0-P1，改变控制点则该函数也需要改变
        double bezier_derivative(Point2d P0, Point2d P1, Point2d P2, double bezier_t);
        // 贝塞尔曲线横移+前进，必须保证前进方向足够的距离，前进与横移右手螺旋
        bool start_bezier(double theta, double dist, double dist_l);
        bool follow_bezier(double theta, double dist, double dist_l, const ros::Time &now);
    };

} // namespace motion_controller

#endif // !_LINE_FOLLOWER_H_