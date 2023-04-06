#pragma once
// ARM_P > ARM_A0
#define ARM_P 14.85
#define ARM_A0 9.8
#define ARM_A1 20.636
#define ARM_A2 13.626
#define ARM_A3 12.524
#define ARM_A4 12.27216
#define ARM_MAX_LEN ((ARM_A0) + (ARM_A2) + (ARM_A3) + (ARM_A4))
#define ARM_MAX_HIGH ((ARM_A1) + (ARM_A2) + (ARM_A3) + (ARM_A4))

#include <ros/ros.h>

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
        Angle(double val);           // 角度值
        Angle(double v1, double v2); // atan2
        double _get_degree();        // 获得角度值
        double rad();                // 转弧度制
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
    double degree(double rad);
    class Axis
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
    public:
        bool expand_y; // 将y的范围扩展到y < 0
        double x, y, z;
        double length();
        double height();
        // 求逆解，输入机械臂角度变量
        bool backward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4, bool look);
        // bool first_step(double &deg1);
        bool test_ok(double &deg1, double &deg2, double &deg3, double &deg4, bool look);
    };
    // 求正解，输入角度，输出位置
    bool forward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4,
                            double &x, double &y, double &z, bool expand_y = false);
}