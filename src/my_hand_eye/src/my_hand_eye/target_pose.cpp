#include "my_hand_eye/target_pose.h"
#include "my_hand_eye/backward_kinematics.h"

namespace my_hand_eye
{
    TargetPose::TargetPose() : target(target_center)
    {
        pose[target_center].theta = Pose2DMightEnd::not_change;
        Action center = Action(0, 32.5, 0).arm2footprint();
        pose[target_center].x = center.x;
        pose[target_center].y = center.y;

        // 偏差必须大于0.01
        tolerance[target_center].theta = Pose2DMightEnd::not_change;
        tolerance[target_center].x = 0.015;
        tolerance[target_center].y = 0.015;

        pose[target_ellipse].theta = Angle(-5.0951).rad();
        // pose[target_ellipse].theta = Angle(-4.0144).rad();
        Action ellipse = Action(0, 19.3, 0).front2left().arm2footprint();
        pose[target_ellipse].x = ellipse.x;
        pose[target_ellipse].y = ellipse.y;

        tolerance[target_ellipse].theta = 0.009;
        // tolerance[target_ellipse].theta = 0.006;
        tolerance[target_ellipse].x = 0.009;
        tolerance[target_ellipse].y = 0.01;
    }

    void TargetPose::calc(geometry_msgs::Pose2D &pose_arm, Pose2DMightEnd &pose_target, const int cnt_max)
    {
        static int err_cnt = 0;  // 防误判
        static int err_cnt2 = 0; // 防不判
        Action a = Action(pose_arm.x, pose_arm.y, 0).arm2footprint();
        if (pose_arm.theta == pose_target.not_change)
            pose_arm.theta = pose[target].theta;
        if (pose_arm.x == pose_target.not_change)
            pose_arm.x = pose[target].x;
        if (pose_arm.y == pose_target.not_change)
            pose_arm.y = pose[target].y;
        pose_target.pose.theta = (pose[target].theta == pose_target.not_change)
                                     ? pose_target.not_change
                                     : (pose_arm.theta - pose[target].theta);
        pose_target.pose.x = (pose[target].x == pose_target.not_change)
                                 ? pose_target.not_change
                                 : (a.x - pose[target].x);
        pose_target.pose.y = (pose[target].y == pose_target.not_change)
                                 ? pose_target.not_change
                                 : (a.y - pose[target].y);
        if (abs(pose_target.pose.theta) <= tolerance[target].theta &&
            abs(pose_target.pose.x) <= tolerance[target].x &&
            abs(pose_target.pose.y) <= tolerance[target].y)
        {
            err_cnt++;
            if ((err_cnt > 2 && target != target_ellipse) || err_cnt > 1)
            {
                pose_target.end = true;
                if (target != target_ellipse || err_cnt > 2 + cnt_max)
                    err_cnt = 0;
                if (err_cnt == 3)
                    err_cnt2 = 0;
            }
            else
                pose_target.end = false;
        }
        else if (target != target_ellipse || err_cnt <= 2)
        {
            pose_target.end = false;
            if (err_cnt)
                err_cnt = 0;
        }
        else if ((++err_cnt2) > 1)
            pose_target.end = false;
        else
        {
            if (err_cnt == 3)
                err_cnt2 = 0;
            err_cnt++;
            pose_target.end = true;
        }
    }
} // namespace my_hand_eye
