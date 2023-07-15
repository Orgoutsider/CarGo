#include "my_hand_eye/target_pose.h"

namespace my_hand_eye
{
    TargetPose::TargetPose()
    {
        pose[target_center].theta = Pose2DMightEnd::not_change;
        pose[target_center].x = 0;
        pose[target_center].y = 0.28;
        tolerance[target_center].theta = Pose2DMightEnd::not_change;
        tolerance[target_center].x = 0.02;
        tolerance[target_center].y = 0.02;
    }

    void TargetPose::calc(Pose2DMightEnd &pme, const Target target)
    {
        // cm转化成m
        pme.pose.theta = (pose[target].theta == pme.not_change)
                             ? pme.not_change
                             : (pme.pose.theta - pose[target].theta);
        pme.pose.x = (pose[target].x == pme.not_change)
                         ? pme.not_change
                         : (pme.pose.x * 0.01 - pose[target].x);
        pme.pose.y = (pose[target].y == pme.not_change)
                         ? pme.not_change
                         : (pme.pose.y * 0.01 - pose[target].y);
        if (abs(pme.pose.theta) <= tolerance[target].theta &&
            abs(pme.pose.x) <= tolerance[target].x && abs(pme.pose.y) <= tolerance[target].y)
        {
            pme.end = true;
        }
        else
            pme.end = false;
    }
} // namespace my_hand_eye
