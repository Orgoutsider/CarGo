#include "motion_controller/vision_adjuster.h"

namespace motion_controller
{
    VisionAdjuster::VisionAdjuster()
    {
        ros::NodeHandle nh;
        usb_cam_subscriber_ = nh.subscribe<std_msgs::Float64>(
            "/vision_usb_cam", 5, &VisionAdjuster::_usb_cam_callback, this);
    }

    void VisionAdjuster::_usb_cam_callback(const std_msgs::Float64ConstPtr &msg)
    {
    }
} // namespace motion_controller
