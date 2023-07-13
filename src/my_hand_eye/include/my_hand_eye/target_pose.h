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
        std::array<geometry_msgs::Pose2D, target_border + 1> pose;
        // 利用目标位姿计算发送的位姿，自动填入not_change
        void calc(Pose2DMightEnd &pme, const Target target);

    private:
        // 允许误差如果对应pose not change则tolerance也为not change
        std::map<Target, geometry_msgs::Pose2D> tolerance;
    };

} // namespace my_hand_eye

#endif // !_TARGET_POSE_H_
