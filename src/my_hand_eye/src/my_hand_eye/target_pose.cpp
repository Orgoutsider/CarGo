#include "my_hand_eye/target_pose.h"

namespace my_hand_eye
{
    TargetPose::TargetPose() : target(target_center)
    {
        pose[target_center].theta = Pose2DMightEnd::not_change;
        pose[target_center].x = 0.325;
        pose[target_center].y = 0;

        // 偏差必须大于0.01
        tolerance[target_center].theta = Pose2DMightEnd::not_change;
        tolerance[target_center].x = 0.015;
        tolerance[target_center].y = 0.015;
    }

    void TargetPose::calc(Pose2DMightEnd &pme_arm, Pose2DMightEnd &pme_target)
    {
        static int err_cnt = 0;
        // cm转化成m并转换坐标系
        pme_target.pose.theta = (pose[target].theta == pme_arm.not_change)
                                    ? pme_arm.not_change
                                    : (pme_arm.pose.theta - pose[target].theta);
        pme_target.pose.x = (pose[target].x == pme_arm.not_change)
                                ? pme_arm.not_change
                                : (pme_arm.pose.y * 0.01 - pose[target].x);
        pme_target.pose.y = (pose[target].y == pme_arm.not_change)
                                ? pme_arm.not_change
                                : (-pme_arm.pose.x * 0.01 - pose[target].y);
        if (abs(pme_target.pose.theta) <= tolerance[target].theta &&
            abs(pme_target.pose.x) <= tolerance[target].x && 
            abs(pme_target.pose.y) <= tolerance[target].y)
        {
            err_cnt++;
            pme_target.pose.theta = pme_target.pose.x = pme_target.pose.y = pme_target.not_change;
            if (err_cnt > 1)
            {
                pme_target.end = true;    
                err_cnt = 0;           
            }
            else
                pme_target.end = false;
        }
        else
        {
            pme_target.end = false;
            if (err_cnt)
                err_cnt = 0;
        }
    }
} // namespace my_hand_eye
