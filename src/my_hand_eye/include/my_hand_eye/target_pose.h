#ifndef _TARGET_POSE_H_
#define _TARGET_POSE_H_

#include <my_hand_eye/Pose2DMightEnd.h>

namespace my_hand_eye
{
    class TargetPose
    {
    public:
        TargetPose();
        enum Target
        {
            target_center,
            target_ellipse,
            target_parking_area,
            target_border
        };
        Target target;
        bool debug;
        std::map<Target, geometry_msgs::Pose2D> pose; // 单位为m
        // 允许误差如果对应pose not change则tolerance也为not change
        std::map<Target, geometry_msgs::Pose2D> tolerance; // 单位为m
        // 利用目标位姿计算发送的位姿，自动填入not_change，返回值表示是否小于误差限
        bool calc(geometry_msgs::Pose2D &pose_arm, Pose2DMightEnd &pose_target, const int cnt_max = 5);
    };

} // namespace my_hand_eye

#endif // !_TARGET_POSE_H_
