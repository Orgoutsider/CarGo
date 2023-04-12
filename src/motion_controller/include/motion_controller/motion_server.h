#ifndef _MOTION_SERVER_H_
#define _MOTION_SERVER_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_controller/MoveAction.h>

namespace motion_controller
{
    typedef actionlib::SimpleActionServer<motion_controller::MoveAction> Server;

    class MotionServer
    {
    private:
        Server server_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        geometry_msgs::PointStamped goal_point_;
        //Optional callback that gets called in a separate thread whenever a new goal is received, allowing users to have blocking callbacks.
        void _execute_call_back(const motion_controller::MoveActionGoalConstPtr &goal);
        void _preempt_call_back();
        bool _add_goal_point(geometry_msgs::Pose2D &pose);

    public:
        MotionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~MotionServer();
    };

} // namespace motion_controller

#endif // !_MOTION_SERVER_H_