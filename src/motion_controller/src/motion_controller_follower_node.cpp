#include "motion_controller/line_follower.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_controller_follower_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    motion_controller::LineFollower follower(nh, pnh);
    ros::spin();
    return 0;
}
