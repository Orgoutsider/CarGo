#ifndef _MOTION_SERVER_H_
#define _MOTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_controller/MoveAction.h>

namespace motion_controller
{
    typedef actionlib::SimpleActionServer<motion_controller::MoveAction> Server;

    class MotionServer
    {
    private:
        int where;
        bool service_on;
    public:
        MotionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~MotionServer();
    };
    
} // namespace motion_controller

#endif // !_MOTION_SERVER_H_