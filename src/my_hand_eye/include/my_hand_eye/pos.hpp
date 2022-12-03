#pragma once
#define ARM_JOINT1_POS_WHEN_DEG0 813.8
#define ARM_JOINT1_POS_WHEN_DEG180 200
#define ARM_JOINT234_POS_WHEN_DEG180 3071.25
#define ARM_JOINT234_POS_WHEN_DEG0 1023.75
#define ARM_JOINT5_POS_WHEN_CATCH 370
#define ARM_JOINT5_POS_WHEN_LOSEN 450
#define ARM_DEFAULT_X 0
#define ARM_DEFAULT_Y 5
#define ARM_DEFAULT_Z 40

#include "nijie.hpp"
#include "my_hand_eye/SCServo.h"
#include "opencv2/opencv.hpp"

namespace nijie{
    struct ArmPose
    {
        cv::Mat R;
        cv::Mat t;
        bool empty;
        ArmPose() : empty(true){};
    };
    
    class Pos : public Axis
    {
    private:
        bool cat;//cat=true抓
        bool look;//look=true观察
        s16 *Position;//均是大小为6的数组
        u16 *Speed;
        u8 *ACC;
        u8 ID[6] = {0, 1, 2, 3, 4, 5};
        SMS_STS *sm_st_ptr;//舵机
        SCSCL *sc_ptr;
    public:
        Pos (s16 *Position, u16 *Speed, u8 *ACC, SMS_STS *sm_st_ptr, SCSCL *sc_ptr, bool cat=false, bool look=true);//初始化
        bool begin(char *argv);//打开串口
        bool calculate_pos();
        //计算各joint运动的position
        bool go_to(double x, double y, double z, bool cat, bool look);//运动到指定位置，抓/不抓
        bool reset();
        bool read_position(int ID);//读指定舵机位置
        bool read_all_position();//读所有舵机正确位置
        bool refresh_xyz();//更新位置
        cv::Mat R_end_to_base();//机械臂末端到基底的·旋转矩阵（不保证实时性）
        cv::Mat T_end_to_base();//机械臂末端到基底的·平移向量（不保证实时性）
        ArmPose end_to_base_now();//更新位置，并返回旋转矩阵，平移向量
        void end();
    };
    bool generate_valid_position(double deg1, double deg2, double deg3, double deg4, double &x, double &y, double &z, bool &look);//根据舵机角度生成位姿
}