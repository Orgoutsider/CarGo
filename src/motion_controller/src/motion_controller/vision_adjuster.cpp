#include "motion_controller/vision_adjuster.h"

namespace motion_controller
{
    VisionAdjuster::VisionAdjuster()
        : level_(level_usb_cam)
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        // usb_cam_subscriber_ = nh.subscribe<Distance>(
        //     "/vision_usb_cam", 5, &VisionAdjuster::_usb_cam_callback, this);
        eye_subscriber_ = nh.subscribe<Pose2DMightEnd>(
            "/vision_eye", 3, &VisionAdjuster::_eye_callback, this);
        cmd_vel_publisher_ = nh.advertise<TwistMightEnd>("/cmd_vel_vision", 3);
        pnh.param<bool>("param_modification", param_modification_, false);
        if (param_modification_)
            dr_server_.setCallback(boost::bind(&VisionAdjuster::_dr_callback, this, _1, _2));
    }

    // void VisionAdjuster::_usb_cam_callback(const DistanceConstPtr &msg)
    // {
    //     if (level_usb_cam == level_)
    //     {
    //         static PIDController pid_usb_cam({0}, {kp_usb_cam_}, {ki_usb_cam_}, {kd_usb_cam_}, {0.03}, {0.1}, {0.5});
    //         if (msg->distance == 0) // distance == 0为标志信号，说明弯道pid完毕
    //         {
    //             TwistMightEnd tme;
    //             tme.end = true;
    //             tme.velocity = geometry_msgs::Twist();
    //             cmd_vel_publisher_.publish(tme);
    //             // 一轮调节完毕，赋值新的PIDController
    //             pid_usb_cam = PIDController({0}, {kp_usb_cam_}, {ki_usb_cam_}, {kd_usb_cam_}, {0.03}, {0.1}, {0.5});
    //             return;
    //         }
    //         std::vector<double> controll;
    //         bool success;
    //         if (pid_usb_cam.update({msg->distance}, msg->header.stamp, controll, success))
    //         {
    //             TwistMightEnd tme;
    //             tme.end = false;
    //             tme.velocity.linear.x = controll[0];
    //             cmd_vel_publisher_.publish(tme);
    //         }
    //     }
    // }

    void VisionAdjuster::_eye_callback(const Pose2DMightEndConstPtr &msg)
    {
        if (level_eye >= level_)
        {
            if (level_eye > level_)
                level_ = level_eye;
            else if (msg->end)
            {
                TwistMightEnd tme;
                tme.end = true;
                cmd_vel_publisher_.publish(tme);
                level_ = level_usb_cam;
                return;
            }
        }
    }

    void VisionAdjuster::_dr_callback(params_PID_visionConfig &config, uint32_t level)
    {
    }
} // namespace motion_controller
