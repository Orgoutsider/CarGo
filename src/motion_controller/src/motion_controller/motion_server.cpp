#include "motion_controller/motion_server.h"

namespace motion_controller
{
    MotionServer::MotionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : server_(nh, "Move", &MotionServer::_execute_call_back, false),
          listener_(buffer_)
    {
        server_.registerPreemptCallback(&MotionServer::_preempt_call_back);
        server_.start();
    }

    MotionServer::~MotionServer()
    {
    }

    void MotionServer::_execute_call_back(const motion_controller::MoveActionGoalConstPtr &goal)
    {
        ros::Rate rate(10);
    }

    void MotionServer::_preempt_call_back()
    {
        ROS_WARN_STREAM("Preempt Requested!");
        server_.setPreempted(MoveResult(), "Got preempted by a new goal");
    }

    bool MotionServer::_add_goal_point(geometry_msgs::Pose2D &pose)
    {
        geometry_msgs::PointStamped point_footprint;
        point_footprint.header.frame_id = "base_footprint";
        point_footprint.header.stamp = ros::Time();
        point_footprint.point.x = pose.x;
        point_footprint.point.y = pose.y;
        point_footprint.point.z = 0;
        try
        {
            goal_point_ = buffer_.transform(point_footprint, "odom_combined");
            // ROS_INFO("坐标点相对于 odom_combined 的坐标为:(%.2f,%.2f,%.2f)",point_base.point.x,point_base.point.y,point_base.point.z);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("Transform error:%s",e.what());
            return false;
        }
        return true;
    }
} // namespace motion_controller
