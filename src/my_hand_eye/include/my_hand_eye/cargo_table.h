#ifndef _CARGO_TABLE_H_
#define _CARGO_TABLE_H_

#define ARM_CARGO_TABLE_POS_WHEN_DEG120 1365

#include <ros/ros.h>

#include "my_hand_eye/SCServo.h"

namespace my_hand_eye
{
    enum Color
    {
        color_red = 1,
        color_green,
        color_blue
    };

    class CargoTable
    {
    private:
        u8 ID;
        u16 Speed;
        u8 ACC;
        int where_;
        int where_last_;
        SMS_STS *sm_st_ptr_;             // 舵机
        std::array<int, 3> what_color_;  // 根据位置找颜色
        std::array<int, 4> where_cargo_; // 根据颜色找位置

    public:
        CargoTable(SMS_STS *sm_st_ptr);
        void set_speed_and_acc(XmlRpc::XmlRpcValue &servo_description); // 获取速度加速度
        void put_next(const Color color);                                       // 移动至下一个位置并记录颜色
        bool is_moving();                                               // 是否运动
        bool arrived(int tolerance = 4);                                                 // 是否到达下一位置
        double calculate_time();                                        // 计算运动时间
        void get_next();                                                // 取下一个位置的物料
        void get_color(const Color color);                                               // 取特定颜色的物料
    };
} // namespace my_hand_eye

#endif // ！_CARGO_TABLE_H_