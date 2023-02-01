#pragma once
#define ARM_JOINT1_POS_WHEN_DEG0 813.8
#define ARM_JOINT1_POS_WHEN_DEG180 200
#define ARM_JOINT234_POS_WHEN_DEG180 3071.25
#define ARM_JOINT234_POS_WHEN_DEG0 1023.75
#define ARM_JOINT5_POS_WHEN_CATCH 370
#define ARM_JOINT5_POS_WHEN_LOSEN 500
#define ARM_INFO_XYZ(Pos) ROS_INFO_STREAM("[" << (Pos).x << ", " << (Pos).y << ", " << (Pos).z << "]")
#include "opencv2/opencv.hpp"

#include "my_hand_eye/SCServo.h"
#include "my_hand_eye/backward_kinematics.hpp"
#include "my_hand_eye/Plot.h"
#include "mmdetection_ros/cargoSrv.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <numeric>

namespace my_hand_eye
{
    struct ArmPose
    {
        cv::Mat R;
        cv::Mat t;
        bool empty;
        ArmPose();
    };

    class Pos : public Axis
    {
    private:
        bool cat;                  // cat=true抓
        bool look;                 // look=true观察
        s16 Position[6] = {0};     // 目标舵机位置
        s16 Position_now[6] = {0}; // 当前舵机位置
        u16 Speed[6] = {0};
        u8 ACC[6] = {0};
        u8 Id[6] = {0, 1, 2, 3, 4, 5};
        SMS_STS *sm_st_ptr; // 舵机
        SCSCL *sc_ptr;
        const double fx = 1097.2826519484;
        const double fy = 1093.696551235277;
        const double cx = 953.2646757356512;
        const double cy = 501.2671482091341;
        cv::Mat R_cam_to_end = (cv::Mat_<double>(3, 3) << 0.01762304284718308, 0.05031655330790039, 0.998577825121317,
                                0.9994569158454911, 0.02692675460875993, -0.01899534824262144,
                                -0.02784424050724299, 0.9983702691634265, -0.04981469583488352);
        // cv::Mat T_cam_to_end = (cv::Mat_<double>(3, 1) << -4.76346677244081, -1.372151631737261, -70.09445432936754);
        cv::Mat T_cam_to_end = (cv::Mat_<double>(3, 1) << -4.76346677244081, -1.372151631737261, -61.00245432936754);
        // cv::Mat T_cam_to_end = (cv::Mat_<double>(3, 1) << -4.76346677244081, -1.372151631737261, -61.50245432936754);
    public:
        Pos(SMS_STS *sm_st_ptr, SCSCL *sc_ptr, bool cat = false, bool look = true); // 初始化
        double default_x, default_y, default_z;
        double put_x, put_y, put_z;
        // double wait_time_;
        bool begin(const char *argv); // 打开串口
        void ping();
        void set_speed_and_acc(XmlRpc::XmlRpcValue &servo_descriptions);            // 获取速度加速度
        void set_action(XmlRpc::XmlRpcValue &action, std::string name = "default"); // 获取设定动作
        // 计算各joint运动的position
        bool calculate_position();
        double calculate_time(int ID);  // 为指定舵机计算到达时间
        bool arrive(int ID[], int IDn); // 判断所有是否到达指定位置附近
        void wait_for_arriving(int ID[], int IDn);
        bool go_to(double x, double y, double z, bool cat, bool look); // 运动到指定位置，抓/不抓
        bool do_first_step(double x, double y);                        // 两步抓取第一步
        bool reset();
        bool go_to_and_wait(double x, double y, double z, bool cat);         // 运动到指定位置，运动完成后抓/不抓
        bool go_to_by_midpoint(double x, double y, double z);                // 通过中间点到达
        bool read_position(int ID);                                          // 读指定舵机位置
        bool read_move(int ID);                                              // 指定舵机运动
        int read_load(int ID);                                               // 读指定舵机负载
        bool show_voltage();                                                 // 显示电压，需要时警告
        bool read_all_position();                                            // 读所有舵机正确位置
        bool is_moving(int ID[], int IDn);                                   // 判断指定舵机运动
        double wait_until_static(int ID[], int IDn, bool show_load = false); // 等待静止，如果show_load为真，返回最大load
        bool refresh_xyz(bool read = true);                                  // 更新位置
        cv::Mat R_end_to_base();                                             // 机械臂末端到基底的·旋转矩阵（不保证实时性）
        cv::Mat T_end_to_base();                                             // 机械臂末端到基底的·平移向量（不保证实时性）
        cv::Mat Intrinsics();                                                // 内参
        ArmPose end_to_base_now();                                           // 更新位置，并返回旋转矩阵，平移向量
        bool calculate_cargo_position(double u, double v, double z, double &x, double &y);
        double distance(double length_goal, double height_goal, double &k); // 中间点位置及移动方向
        bool dfs_midpoint(double length_goal, double height_goal);
        // 求中间点(y > 0)，与calculate_position配合使用，中间点-Position,x,y,z，最终点-Position_goal,x_goal,y_goal,z_goal
        bool find_a_midpoint(s16 Position_goal[], double &x_goal, double &y_goal, double &z_goal);
        void get_points(double h, my_hand_eye::PointArray &arr);
        bool find_points_with_height(double h, my_hand_eye::PointArray &arr);
        void wait_for_alpha_decrease(double alpha_bound); // 等待alpha减小指定值
        void end();
    };

    bool generate_valid_position(double deg1, double deg2, double deg3, double deg4, double &x, double &y, double &z, bool &look); // 根据舵机角度生成位姿
    cv::Mat R_T2homogeneous_matrix(const cv::Mat &R, const cv::Mat &T);
}