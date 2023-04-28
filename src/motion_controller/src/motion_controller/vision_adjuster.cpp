#include "motion_controller/vision_adjuster.h"

namespace motion_controller
{
    VisionAdjuster::VisionAdjuster()
    {
        ros::NodeHandle nh;
        usb_cam_subscriber_ = nh.subscribe<std_msgs::Float64>(
            "/vision_usb_cam", 5, &VisionAdjuster::_usb_cam_callback, this);
        eye_subscriber_ = nh.subscribe<geometry_msgs::Pose2D>(
            "/vision_eye", 3, &VisionAdjuster::_eye_callback, this);
        cmd_vel_publisher_ = nh.advertise<TwistMightEnd>("/cmd_vel_vision", 3);
    }

    void VisionAdjuster::_usb_cam_callback(const std_msgs::Float64ConstPtr &msg)
    {
    }
} // namespace motion_controller
