#ifndef _POSE_H_
#define _POSE_H_

#define ARM_JOINT1_POS_WHEN_DEG180 500
#define ARM_JOINT1_POS_WHEN_DEG0 1082
#define ARM_JOINT2_POS_WHEN_DEG180 3072
#define ARM_JOINT2_POS_WHEN_DEG0 1024
#define ARM_JOINT3_POS_WHEN_DEG180 3072
#define ARM_JOINT3_POS_WHEN_DEG0 1024
#define ARM_JOINT4_POS_WHEN_DEG180 3072
#define ARM_JOINT4_POS_WHEN_DEG0 1024
#define ARM_JOINT5_POS_WHEN_CATCH 677
#define ARM_JOINT5_POS_WHEN_OPEN 880
#define ARM_INFO_XYZ(Pos) ROS_INFO_STREAM("[" << (Pos).x << ", " << (Pos).y << ", " << (Pos).z << "]")
#define ARM_WARN_XYZ(Pos) ROS_WARN_STREAM("[" << (Pos).x << ", " << (Pos).y << ", " << (Pos).z << "]")
#define ARM_ERROR_XYZ(Pos) ROS_ERROR_STREAM("[" << (Pos).x << ", " << (Pos).y << ", " << (Pos).z << "]")
#include <opencv2/opencv.hpp>
#include "my_hand_eye/backward_kinematics.h"
#include "my_hand_eye/cargo_table.h"
#include <my_hand_eye/Plot.h>

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

    Action front2left(Action &ac);

    class Pos : public Axis
    {
    private:
        bool look_;          // look=true观察
        s16 Position[6];     // 目标舵机位置
        s16 Position_now[6]; // 当前舵机位置
        u16 Speed[6];
        u8 ACC[6];
        u8 Id[6];
        SMS_STS *sm_st_ptr_; // 舵机
        SCSCL *sc_ptr_;
        CargoTable cargo_table_;
        const double fx = 788.709302;
        const double fy = 940.728627;
        const double cx = 932.106780;
        const double cy = 578.390364;
        double calculate_time(int ID); // 为指定舵机计算到达时间
        bool arrived(u8 ID[], u8 IDN); // 判断所有是否到达指定位置附近
        // 计算各joint运动的position
        bool calculate_position(bool expand_y = false);
        bool read_position(int ID);      // 读指定舵机位置
        bool read_move(int ID);          // 指定舵机运动
        int read_load(int ID);           // 读指定舵机负载
        bool is_moving(u8 ID[], u8 IDN); // 判断指定舵机运动
        // 等待静止且到达指定位置附近，如果show_load为真，返回最大load
        double wait_until_static(u8 ID[], u8 IDN, bool show_load = false);
        cv::Mat R_cam_to_end();
        cv::Mat T_cam_to_end();
        // 在y > 0处使用
        cv::Mat R_end_to_base();                                            // 机械臂末端到基底的旋转矩阵（不保证实时性）
        cv::Mat T_end_to_base();                                            // 机械臂末端到基底的平移向量（不保证实时性）
        cv::Mat intrinsics();                                               // 内参
        cv::Mat intrinsics_inverse();                                       // 内参的逆矩阵
        cv::Mat extrinsics();                                               // 外参（不保证实时性，配合refresh_xyz）
        cv::Mat extrinsics_inverse();                                       // 外参的逆矩阵（不保证实时性，配合refresh_xyz）
        double distance(double length_goal, double height_goal, double &k); // 中间点位置及移动方向
        bool dfs_midpoint(double length_goal, double height_goal);
        void get_points(double h, my_hand_eye::PointArray &arr);

    public:
        double tightness;   // 取值0～1，为0时最松，为1时最紧
        Pos(SMS_STS *sm_st_ptr, SCSCL *sc_ptr, bool cat = false, bool look = true); // 初始化
        Action action_default;
        Action action_left;
        Action action_back;
        Action action_right;
        Action action_down;
        // double wait_time_;
        bool begin(const char *argv); // 打开串口
        void ping();
        void set_speed_and_acc(XmlRpc::XmlRpcValue &servo_descriptions);                      // 获取速度加速度
        void set_action(XmlRpc::XmlRpcValue &action, std::string name = "default");           // 获取设定动作
        bool go_to(double x, double y, double z, bool cat, bool look, bool expand_y = false); // 运动到指定位置，抓/不抓
        // bool do_first_step(double x, double y);                        // 两步抓取第一步
        bool reset(bool left = false);                               // 重置位置，可选前侧/左侧
        bool look_down();                                            // 查看左侧车道线
        bool go_to_and_wait(double x, double y, double z, bool cat); // 运动到指定位置，运动完成后抓/不抓
        bool go_to_by_midpoint(double x, double y, double z);        // 通过中间点到达
        bool go_to_table(bool cat, Color color, bool left);
        bool show_voltage();                     // 显示电压，需要时警告
        bool read_all_position();                // 读所有舵机正确位置
        bool refresh_xyz(bool read = true);      // 更新位置
        ArmPose end_to_base_now();               // 更新位置，并返回旋转矩阵，平移向量
        cv::Mat transformation_matrix(double z); // 透视变换矩阵（不保证实时性，配合refresh_xyz）
        // 计算物料位置
        bool calculate_cargo_position(double u, double v, double cargo_z,
                                      double &cargo_x, double &cargo_y, bool read = true);
        // 由实际位置计算像素位置
        bool calculate_pixel_position(double cargo_x, double cargo_y, double cargo_z,
                                      double &u, double &v, bool read);
        // 计算边界线位置
        bool calculate_border_position(cv::Vec2f &border, double border_z,
                                       double &distance, double &yaw);
        // 通过记录的位置校正外参
        bool extrinsics_correction(double u, double v,
                                   double correct_x, double correct_y, double correct_z);
        // 求中间点(y > 0)，与calculate_position配合使用，中间点-Position,x,y,z，最终点-Position_goal,x_goal,y_goal,z_goal
        bool find_a_midpoint(s16 Position_goal[], double &x_goal, double &y_goal, double &z_goal);
        bool find_points_with_height(double h, my_hand_eye::PointArray &arr);
        void wait_for_alpha_decrease(double alpha_bound); // 等待alpha减小指定值
        void end();
    };

    bool generate_valid_position(double deg1, double deg2, double deg3, double deg4,
                                 double &x, double &y, double &z, bool &look); // 根据舵机角度生成位姿
    // 生成齐次矩阵
    cv::Mat R_T2homogeneous_matrix(const cv::Mat &R, const cv::Mat &T);
}

#endif // !_POSE_H_
