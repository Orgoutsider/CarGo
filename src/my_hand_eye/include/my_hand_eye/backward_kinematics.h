#ifndef _BACKWARD_KINEMATICS_H_
#define _BACKWARD_KINEMATICS_H_

#include <ros/ros.h>

// ARM_P > ARM_A0
#define ARM_P 14.006
#define ARM_A0 9.8
#define ARM_A1 19.445
#define ARM_A2 13.626
#define ARM_A3 12.524
#define ARM_A4 13.06707
#define ARM_A5 0.025
#define ARM_MAX_LEN ((ARM_A0) + (ARM_A2) + (ARM_A3) + sqrt((ARM_A4) * (ARM_A4) + (ARM_A5) * (ARM_A5)))
#define ARM_MAX_HIGH ((ARM_A1) + (ARM_A2) + (ARM_A3) + sqrt((ARM_A4) * (ARM_A4) + (ARM_A5) * (ARM_A4)))
#define ARM_INFO_XYZ(action) ROS_INFO_STREAM("[" << (action).x << ", " << (action).y << ", " << (action).z << "]")
#define ARM_WARN_XYZ(action) ROS_WARN_STREAM("[" << (action).x << ", " << (action).y << ", " << (action).z << "]")
#define ARM_ERROR_XYZ(action) ROS_ERROR_STREAM("[" << (action).x << ", " << (action).y << ", " << (action).z << "]")

namespace my_hand_eye
{
    class Angle
    {
    private:
        double deg;
        int state; // 目前角度计算是否出错

    public:
        const int error = 1;
        const int normal = 0;
        Angle(double deg);                        // 角度值
        static Angle atan2(double v1, double v2); // atan2
        double _get_degree();                     // 获得角度值
        double rad();                             // 转弧度制
        static double degree(double rad);         // 转角度制
        double cos();
        double sin();
        void _j_degree_convert(int joint);     // 将j1-j4和机械臂的角度互换
        bool _valid_degree(int joint);         // 判断机械臂角度是否在合理范围
        bool _valid_j(int joint);              // 判断j1-j4是否在合理范围
        void _set_state(int);                  // 设置状态
        Angle operator+(const Angle &t) const; // 重载加法
        Angle operator-(const Angle &t) const; // 重载减法
        Angle operator-() const;               // 重载负号
        Angle operator=(const Angle &t);       // 重载等号
        bool operator>(const Angle &t);        // 重载>
        bool operator<(const Angle &t);        // 重载<
    };

    class Action
    {
    public:
        double x, y, z;
        Action();
        Action(double x, double y, double z);
        Action front2left();
        // cm转化成m并转换坐标系
        Action arm2footprint();
        // 补偿与目标位置的误差
        Action now2goal(double err_x, double err_y, double err_theta, Action enlarge);
        Action operator+=(const Action &t); // 重载加法
        Action operator-=(const Action &t); // 重载减法
    protected:
        double length();
        double height();
        // static double normxy(const Action &a1, const Action &a2);
    };

    class Axis : public Action
    {
    private:
        double L(double alpha);
        double H(double alpha);
        bool _out_of_range();
        Angle _calculate_j1();
        Angle _calculate_j3(double alpha);
        Angle _calculate_j2(double alpha);
        Angle _calculate_j4(double alpha);
        bool _j123_length_and_height_is_valid(double alpha); // j123和alpha是否合理
        bool _modify_xy();                                   // 如果expand_y为真，则将x与y进行调整，必须调用2次
        bool _modify_alpha(double &alpha, bool look);        // 自动调整alpha，look表示是否为观察模式
    protected:
        bool expand_y; // 将y的范围扩展到y < 0
    public:
        Axis();
        // 求逆解，输入机械臂角度变量
        bool backward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4, bool look);
        // bool first_step(double &deg1);
        bool test_ok(double &deg1, double &deg2, double &deg3, double &deg4, bool look);
    };
    // 求正解，输入角度，输出位置
    bool forward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4,
                            double &x, double &y, double &z, bool expand_y = false);
}

#endif // !_BACKWARD_KINEMATICS_H_