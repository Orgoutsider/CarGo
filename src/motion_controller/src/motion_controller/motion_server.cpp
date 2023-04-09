#include "motion_controller/motion_server.h"

namespace motion_controller
{
    MotionServer::MotionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : server_(nh, "Move", &MotionServer::call_back, false)
    {
    }

    MotionServer::~MotionServer()
    {
    }
} // namespace motion_controller
