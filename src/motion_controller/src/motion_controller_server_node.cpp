#include "motion_controller/motion_server.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_controller_server_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    motion_controller::MotionServer server(nh, pnh);
    ros::spin();
    return 0;
}
