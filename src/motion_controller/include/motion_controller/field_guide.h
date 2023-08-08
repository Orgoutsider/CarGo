#ifndef _FIELD_GUIDE_H_
#define _FIELD_GUIDE_H_

#include <ros/ros.h>
#include <motion_controller/routeConfig.h>

namespace motion_controller
{
    class FieldGuide
    {
    private:
        std::array<int, 14> route_; // 赛道所有场景数组
        bool doing_;                // 是否正在运行机械臂任务
        int where_;                 // route索引
        double length_field_;       // 场地（车道部分）长宽
        double x_raw_material_area_;
        double y_roughing_area_;
        double x_semi_finishing_area_;

    public:
        int dr_route_; // 调参时面向的场景
        double x_;
        double y_;
        double theta_;               // 位姿
        bool left_;                  // 移动方向，是否逆时针（左转）
        int loop_;                   // 当前为第几轮
        double length_car_;          // 车长
        double width_road_;          // 路宽
        double length_parking_area_; // 停车区长宽
        double y_road_up_up_;        // 从停车区上侧挡板到上路上沿
        double x_QR_code_board_;     // 二维码板
        double y_parking_area_;      // 机械臂开始运动识别停车区的坐标，和停车区有一段距离
        FieldGuide();
        int where_is_car(bool debug, bool startup = false) const;
        // 当前任务正在完成，不可接下一任务
        void doing();
        // 当前任务已完成，接下一任务，更新loop_
        void finished();
        bool arrived() const;
        // 下一路段没有任务，可以直接转弯
        bool can_turn() const;
        // 位于任务点所在道路，距离下一任务点的距离
        double length_route() const;
        // 位于弯道，到弯道中心线的距离，不位于弯道时返回0
        double length_corner() const;
        // 位于弯道，转弯角度，不位于弯道时返回0
        double angle_corner() const;
        // 掉头角度
        double angle_U_turn() const;
    };
} // namespace motion_controller

#endif // _FIELD_GUIDE_H_