#ifndef _FIELD_GUIDE_H_
#define _FIELD_GUIDE_H_

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <motion_controller/routeConfig.h>

namespace motion_controller
{
    class FieldGuide
    {
    private:
        std::array<int, 13> route_; // 赛道所有场景数组
        bool doing_;                // 是否正在运行机械臂任务
        int where_;                 // route索引

    public:
        int dr_route_; // 调参时面向的场景
        double x_;
        double y_;
        double theta_;        // 位姿
        int loop_;            // 当前为第几轮
        double length_car_;   // 车长
        double width_car_;    // 车宽
        double width_road_;   // 路宽
        double length_field_; // 场地（车道部分）长
        double width_field_;  // 场地（车道部分）宽
        double y_raw_material_area_;
        double x_roughing_area_;
        double length_from_semi_finishing_area_; // 左边界到半成品区距离
        double y_semi_finishing_area_;
        double length_from_ellipse_;            // 到中心椭圆距离道路方向
        double width_from_semi_finishing_area_; // 到半成品区距离宽度方向
        double width_from_roughing_area_;       // 到粗加工区椭圆距离宽度方向
        double y_palletize_;                    // 码垛时纠正位置
        double width_palletize_;                // 到半成品区（码垛）距离宽度方向
        double length_from_parking_area_;       // 到停车区距离x
        double width_from_parking_area_;        // 到停车区距离y
        double length_parking_area_;            // 停车区边长
        double x_road_up_;                      // 从停车区上侧挡板到上路上沿
        double y_QR_code_board_;                // 二维码板
        double x_QR_code_board_;                // 为扫描二维码不行驶在路中心，而是距离车道线一定距离
        double angle_raw_material_area_;        // 位于原料区旋转的角度
        double radius_raw_material_area_;       // 转盘半径
        double x_parking_area_;                 // 机械臂开始运动识别停车区的坐标，和停车区有一段距离
        bool clockwise_;                        // 是否顺时针移动
        FieldGuide();
        int where_is_car(bool debug, bool startup = false, int offset = 0) const;
        // 当前任务正在完成，不可接下一任务
        void doing(boost::recursive_mutex &mtx);
        // 当前任务已完成，接下一任务，更新loop_
        void finished(boost::recursive_mutex &mtx);
        bool is_doing() const;
        bool arrived(bool debug, bool startup = false) const;
        // 位于任务点所在道路，距离下一任务点的距离，正号表示沿逆时针
        double length_route(bool debug, bool startup = false, int offset = 0) const;
        // // 位于弯道，到弯道中心线的距离，不位于弯道时返回0.5，正号表示沿y轴正向，已弃用
        // double length_border() const;
        // 偏离道路中心的距离
        double length_from_road(bool debug, bool startup = false, int offset = 0) const;
        // 偏离道路中心的角度
        double angle_from_road(bool debug, bool startup = false, int offset = 0) const;
        // 在转弯处设置的位置，指定是否对外围黄色区域
        bool position_in_corner(double dist, double yaw,
                                double &x, double &y, double &theta, bool outside = true) const;
        // 转弯角度
        double angle_corner() const;
        // 角度校正
        double angle_correction(double theta) const;
        // // 下一路段没有任务，可以直接转弯
        // bool can_turn() const;
        // // 掉头角度
        // double angle_U_turn() const;
    };
} // namespace motion_controller

#endif // _FIELD_GUIDE_H_